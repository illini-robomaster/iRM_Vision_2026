#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cerrno>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <future>
#include <mutex>
#include <nlohmann/json.hpp>
#include <thread>

#include "calibration/intrinsic_session.hpp"
#include "io/camera.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"

namespace fs = std::filesystem;
using Json = nlohmann::json;
using namespace calibration;

namespace
{
const char * keys =
  "{help h | | Show help}"
  "{config c | configs/mv_sua133gc.yaml | Camera YAML; no CBoard required}"
  "{port p | 8081 | HTTP port}"
  "{bind b | 127.0.0.1 | IPv4 bind address; use 0.0.0.0 only on trusted LAN}"
  "{output o | records/calibration | Session output root}"
  "{images i | | Offline image directory, sorted by filename; no camera opened}";

std::string read_text(const std::string & path)
{
  std::ifstream file(path);
  if (!file) throw std::runtime_error("Cannot read " + path);
  return {std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
}

void write_text(const std::string & path, const std::string & text)
{
  std::ofstream file(path);
  file << text;
  if (!file) throw std::runtime_error("Cannot write " + path);
}

std::string session_directory(const std::string & root)
{
  fs::create_directories(root);
  const auto now = std::time(nullptr);
  const auto tm = *std::localtime(&now);
  char text[64];
  std::strftime(text, sizeof(text), "%Y-%m-%d_%H-%M-%S", &tm);
  for (int i = 0; i < 10000; ++i) {
    auto path = fs::path(root) / (std::string(text) + "_" + std::to_string(i));
    if (fs::create_directory(path)) return fs::absolute(path).string();
  }
  throw std::runtime_error("Cannot allocate unique session directory");
}

struct State
{
  std::mutex mutex;
  Pattern pattern;
  cv::Mat frame;
  std::vector<cv::Point2f> points;
  Timestamp timestamp;
  bool found = false;
  bool connected = false;
  bool busy = false;
  std::string error;
  std::vector<Sample> samples;
  std::optional<Result> result;
  std::future<Result> job;
  int next_id = 1;
  std::string directory;
  // Future hand-eye adapter injection point. Intrinsic mode leaves this null.
  std::unique_ptr<PoseProvider> poses;

  void update(const cv::Mat & image, Timestamp time)
  {
    if (image.empty()) throw std::runtime_error("Empty input frame");
    if (!samples.empty() && image.size() != samples.front().image.size())
      throw std::runtime_error("Resolution changed; restart with consistent camera settings");
    frame = image.clone();
    timestamp = time;
    connected = true;
    found = detect(frame, pattern, points);
  }

  void finish_job()
  {
    if (!busy || job.wait_for(std::chrono::seconds(0)) != std::future_status::ready) return;
    try {
      auto value = job.get();
      write_text(directory + "/intrinsics.yaml", value.yaml());
      result = std::move(value);
      error.clear();
    } catch (const std::exception & e) {
      error = e.what();
    }
    busy = false;
  }

  void invalidate()
  {
    result.reset();
    fs::remove(directory + "/intrinsics.yaml");
  }

  Json status() const
  {
    Json views = Json::array();
    std::array<bool, 9> coverage{};
    for (size_t i = 0; i < samples.size(); ++i) {
      const auto & sample = samples[i];
      Json view = {{"id", sample.id}, {"has_pose", bool(sample.orientation_wxyz)}};
      if (result) {
        view["mean_px"] = result->per_view_mean.at(i);
        view["rms_px"] = result->per_view_rms.at(i);
      }
      views.push_back(view);
      cv::Point2f center(0, 0);
      for (const auto & point : sample.points) center += point;
      center *= 1.0f / sample.points.size();
      int x = std::max(0, std::min(2, int(center.x * 3 / sample.image.cols)));
      int y = std::max(0, std::min(2, int(center.y * 3 / sample.image.rows)));
      coverage[y * 3 + x] = true;
    }
    Json response = {
      {"connected", connected},
      {"found", found},
      {"busy", busy},
      {"error", error},
      {"width", frame.cols},
      {"height", frame.rows},
      {"pattern",
       {{"cols", pattern.cols}, {"rows", pattern.rows}, {"spacing_mm", pattern.spacing_mm}}},
      {"samples", views},
      {"coverage", coverage},
      {"directory", directory},
      {"handeye_available", false},
      {"pose_provider_available", bool(poses)}};
    response["result"] = result ? Json(YAML_to_json(*result)) : Json(nullptr);
    return response;
  }

  static Json YAML_to_json(const Result & value)
  {
    return {{"camera_matrix", std::vector<double>(value.camera_matrix.begin<double>(),
                                                  value.camera_matrix.end<double>())},
            {"distort_coeffs", std::vector<double>(value.distort_coeffs.begin<double>(),
                                                   value.distort_coeffs.end<double>())},
            {"rms_px", value.rms},
            {"mean_px", value.mean_error}};
  }
};

bool send_all(int fd, const std::string & data)
{
  size_t offset = 0;
  while (offset < data.size()) {
    auto count = ::send(fd, data.data() + offset, data.size() - offset, MSG_NOSIGNAL);
    if (count < 0 && errno == EINTR) continue;
    if (count <= 0) return false;
    offset += count;
  }
  return true;
}

void reply(int fd, int code, const std::string & type, const std::string & body)
{
  const std::string reason = code == 200 ? "OK" : code == 404 ? "Not Found" : "Bad Request";
  send_all(
    fd, "HTTP/1.1 " + std::to_string(code) + " " + reason +
          "\r\nConnection: close\r\nCache-Control: no-store\r\nX-Content-Type-Options: nosniff\r\n"
          "Content-Type: " +
          type + "\r\nContent-Length: " + std::to_string(body.size()) + "\r\n\r\n");
  send_all(fd, body);
}

void serve(int fd, State & state, const std::string & page, const std::string & script,
           const std::vector<fs::path> & inputs, size_t & input_index)
{
  std::string request;
  char buffer[2048];
  size_t end;
  while ((end = request.find("\r\n\r\n")) == std::string::npos) {
    auto n = recv(fd, buffer, sizeof(buffer), 0);
    if (n <= 0) return;
    request.append(buffer, n);
    if (request.size() > 8192) throw std::runtime_error("Request too large");
  }
  const auto line_end = request.find("\r\n");
  const auto first = request.substr(0, line_end);
  const auto space = first.find(' ');
  const auto second = first.find(' ', space + 1);
  if (space == std::string::npos || second == std::string::npos)
    throw std::runtime_error("Invalid request line");
  const auto method = first.substr(0, space);
  auto path = first.substr(space + 1, second - space - 1);
  path = path.substr(0, path.find('?'));
  Json body = Json::object();
  if (method == "POST") {
    // JSON-only mutation prevents cross-origin HTML form requests. No CORS is enabled.
    std::string headers = request.substr(0, end);
    std::transform(headers.begin(), headers.end(), headers.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    if (headers.find("content-type: application/json") == std::string::npos)
      throw std::runtime_error("POST requires application/json");
    const auto pos = headers.find("content-length:");
    if (pos == std::string::npos || headers.find("transfer-encoding:") != std::string::npos)
      throw std::runtime_error("Content-Length required; chunked requests unsupported");
    const auto length = std::stoul(headers.substr(pos + 15));
    if (length > 4096) throw std::runtime_error("Body too large");
    while (request.size() < end + 4 + length) {
      auto n = recv(fd, buffer, sizeof(buffer), 0);
      if (n <= 0) throw std::runtime_error("Incomplete request");
      request.append(buffer, n);
    }
    body = Json::parse(request.substr(end + 4, length));
    if (!body.is_object()) throw std::runtime_error("JSON object required");
  } else if (method != "GET") {
    throw std::runtime_error("Unsupported method");
  }
  if (method == "GET" && path == "/") {
    reply(fd, 200, "text/html; charset=utf-8", page);
    return;
  }
  if (method == "GET" && path == "/app.js") {
    reply(fd, 200, "application/javascript; charset=utf-8", script);
    return;
  }
  std::unique_lock<std::mutex> lock(state.mutex);
  state.finish_job();
  if (method == "GET" && path == "/api/status") {
    auto status = state.status();
    status["offline"] = !inputs.empty();
    status["input_index"] = input_index;
    status["input_count"] = inputs.size();
    lock.unlock();
    reply(fd, 200, "application/json", status.dump());
  } else if (method == "GET" && (path == "/preview.jpg" || path == "/sample.jpg")) {
    cv::Mat image;
    if (path == "/sample.jpg") {
      // ID is read from the original query, never interpreted as a filesystem path.
      auto query = first.find("?id=");
      if (query == std::string::npos) throw std::runtime_error("Sample ID required");
      int id = std::stoi(first.substr(query + 4));
      for (const auto & sample : state.samples)
        if (sample.id == id) image = sample.image.clone();
    } else {
      image = state.frame.clone();
      if (!image.empty()) {
        if (first.find("undistort=1") != std::string::npos && state.result) {
          cv::Mat corrected;
          cv::undistort(image, corrected, state.result->camera_matrix,
                        state.result->distort_coeffs);
          image = corrected;
        } else {
          cv::drawChessboardCorners(image, {state.pattern.cols, state.pattern.rows}, state.points,
                                    state.found);
        }
      }
    }
    if (image.empty()) throw std::runtime_error("No image available");
    lock.unlock();
    const double scale = std::min(1.0, 960.0 / image.cols);
    cv::resize(image, image, {}, scale, scale);
    std::vector<uchar> jpeg;
    cv::imencode(".jpg", image, jpeg, {cv::IMWRITE_JPEG_QUALITY, 80});
    reply(fd, 200, "image/jpeg", std::string(jpeg.begin(), jpeg.end()));
  } else if (method == "GET" && path == "/intrinsics.yaml") {
    if (!state.result) throw std::runtime_error("No current calibration result");
    const auto yaml = state.result->yaml();
    lock.unlock();
    reply(fd, 200, "application/yaml", yaml);
  } else if (method == "POST") {
    if (state.busy) throw std::runtime_error("Calibration running; sample changes are locked");
    if (path == "/api/capture") {
      if (!state.connected || !state.found)
        throw std::runtime_error("Complete circle grid not detected");
      if (state.samples.size() >= 100) throw std::runtime_error("Maximum 100 samples");
      if (inputs.empty() &&
          std::chrono::steady_clock::now() - state.timestamp > std::chrono::seconds(3))
        throw std::runtime_error("Frame is stale");
      Sample sample;
      sample.id = state.next_id++;
      sample.image = state.frame.clone();
      sample.points = state.points;
      sample.timestamp = state.timestamp;
      if (state.poses) sample.orientation_wxyz = state.poses->orientation_at(sample.timestamp);
      if (!cv::imwrite(state.directory + "/" + std::to_string(sample.id) + ".png", sample.image))
        throw std::runtime_error("Cannot save raw sample");
      state.invalidate();
      state.samples.push_back(std::move(sample));
      save_manifest(state.directory, state.samples, state.pattern);
    } else if (path == "/api/delete") {
      int id = body.at("id").get<int>();
      auto it = std::find_if(state.samples.begin(), state.samples.end(),
                             [id](const Sample & sample) { return sample.id == id; });
      if (it == state.samples.end()) throw std::runtime_error("Unknown sample ID");
      fs::remove(state.directory + "/" + std::to_string(id) + ".png");
      state.invalidate();
      state.samples.erase(it);
      save_manifest(state.directory, state.samples, state.pattern);
    } else if (path == "/api/pattern") {
      Pattern pattern;
      pattern.cols = body.at("cols").get<int>();
      pattern.rows = body.at("rows").get<int>();
      pattern.spacing_mm = body.at("spacing_mm").get<double>();
      pattern.validate();
      if (!state.samples.empty() && !body.value("clear", false))
        throw std::runtime_error("Confirm clearing samples before changing pattern");
      for (const auto & sample : state.samples)
        fs::remove(state.directory + "/" + std::to_string(sample.id) + ".png");
      state.invalidate();
      state.samples.clear();
      state.pattern = pattern;
      state.found = detect(state.frame, state.pattern, state.points);
      save_manifest(state.directory, state.samples, state.pattern);
    } else if (path == "/api/calibrate") {
      if (state.samples.size() < 5)
        throw std::runtime_error("Collect at least 5 views; 15-25 diverse views recommended");
      auto samples = state.samples;
      auto pattern = state.pattern;
      state.invalidate();
      state.error.clear();
      state.job =
        std::async(std::launch::async, [samples, pattern] { return calibrate(samples, pattern); });
      state.busy = true;
    } else if (path == "/api/next") {
      if (inputs.empty() || input_index + 1 >= inputs.size())
        throw std::runtime_error("No next offline image");
      auto image = cv::imread(inputs[input_index + 1].string());
      state.update(image, std::chrono::steady_clock::now());
      ++input_index;
    } else {
      reply(fd, 404, "application/json", "{\"error\":\"Unknown route\"}");
      return;
    }
    lock.unlock();
    reply(fd, 200, "application/json", "{\"ok\":true}");
  } else {
    reply(fd, 404, "application/json", "{\"error\":\"Unknown route\"}");
  }
}
}  // namespace

int main(int argc, char ** argv)
{
  int listener = -1;
  std::atomic<bool> stop{false};
  std::thread capture;
  // State outlives capture thread even when main setup throws.
  State state;
  try {
    cv::CommandLineParser cli(argc, argv, keys);
    if (cli.has("help")) {
      cli.printMessage();
      return 0;
    }
    const int port = cli.get<int>("port");
    const auto bind = cli.get<std::string>("bind");
    if (!cli.check() || port < 1 || port > 65535) throw std::runtime_error("Invalid CLI arguments");
    const auto config = YAML::LoadFile(cli.get<std::string>("config"));
    if (config["pattern_cols"]) state.pattern.cols = config["pattern_cols"].as<int>();
    if (config["pattern_rows"]) state.pattern.rows = config["pattern_rows"].as<int>();
    if (config["center_distance_mm"])
      state.pattern.spacing_mm = config["center_distance_mm"].as<double>();
    state.pattern.validate();
    const auto page = read_text(std::string(CALIBRATION_WEB_DIR) + "/index.html");
    const auto script = read_text(std::string(CALIBRATION_WEB_DIR) + "/app.js");
    std::vector<fs::path> inputs;
    const auto folder = cli.get<std::string>("images");
    if (!folder.empty()) {
      for (const auto & entry : fs::directory_iterator(folder)) {
        const auto extension = entry.path().extension().string();
        if (entry.is_regular_file() &&
            (extension == ".png" || extension == ".jpg" || extension == ".jpeg"))
          inputs.push_back(entry.path());
      }
      std::sort(inputs.begin(), inputs.end());
      if (inputs.empty()) throw std::runtime_error("No images in offline input directory");
      state.update(cv::imread(inputs.front().string()), std::chrono::steady_clock::now());
    }
    listener = socket(AF_INET, SOCK_STREAM, 0);
    if (listener < 0) throw std::runtime_error("Cannot create HTTP socket");
    int yes = 1;
    setsockopt(listener, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
    timeval timeout{0, 200000};
    setsockopt(listener, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(port);
    if (inet_pton(AF_INET, bind.c_str(), &address.sin_addr) != 1 ||
        ::bind(listener, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0 ||
        listen(listener, 8) < 0)
      throw std::runtime_error("Cannot bind/listen on requested IPv4 address/port");
    state.directory = session_directory(cli.get<std::string>("output"));
    save_manifest(state.directory, state.samples, state.pattern);
    tools::Exiter exiter;
    if (inputs.empty()) {
      const auto path = cli.get<std::string>("config");
      capture = std::thread([&state, &stop, path] {
        try {
          io::Camera camera(path);
          while (!stop) {
            cv::Mat image;
            Timestamp timestamp;
            camera.read(image, timestamp);
            {
              std::lock_guard<std::mutex> lock(state.mutex);
              state.update(image, timestamp);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
          }
        } catch (const std::exception & e) {
          std::lock_guard<std::mutex> lock(state.mutex);
          state.connected = false;
          state.error = e.what();
          tools::logger()->error("[IntrinsicGUI] Camera: {}", e.what());
        }
      });
    }
    tools::logger()->info("[IntrinsicGUI] http://{}:{}/ ; output={} ; CBoard disabled", bind, port,
                          state.directory);
    size_t input_index = 0;
    while (!exiter.exit()) {
      int fd = accept(listener, nullptr, nullptr);
      if (fd < 0) continue;
      timeval client_timeout{1, 0};
      setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &client_timeout, sizeof(client_timeout));
      setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &client_timeout, sizeof(client_timeout));
      try {
        serve(fd, state, page, script, inputs, input_index);
      } catch (const std::exception & e) {
        reply(fd, 400, "application/json", Json({{"error", e.what()}}).dump());
      }
      close(fd);
    }
    stop = true;
    if (capture.joinable()) capture.join();
    if (state.busy) {
      state.job.wait();
      state.finish_job();
    }
    close(listener);
    tools::logger()->info("[IntrinsicGUI] Closed normally; camera released");
    return 0;
  } catch (const std::exception & e) {
    stop = true;
    if (capture.joinable()) capture.join();
    if (listener >= 0) close(listener);
    tools::logger()->error("[IntrinsicGUI] {}", e.what());
    return 1;
  }
}