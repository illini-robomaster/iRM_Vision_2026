#ifndef TOOLS__YAML_HPP
#define TOOLS__YAML_HPP

#include <string>

#include <yaml-cpp/yaml.h>

#include "tools/logger.hpp"

namespace tools
{
inline YAML::Node load(const std::string & path)
{
  try {
    return YAML::LoadFile(path);
  } catch (const YAML::BadFile & e) {
    logger()->error("[YAML] Failed to load file: {}", e.what());
    exit(1);
  } catch (const YAML::ParserException & e) {
    logger()->error("[YAML] Parser error: {}", e.what());
    exit(1);
  }
}

template <typename T>
inline T read(const YAML::Node & yaml, const std::string & key)
{
  if (yaml[key]) return yaml[key].as<T>();
  logger()->error("[YAML] {} not found!", key);
  exit(1);
}

// 下面是「可选键」读取：缺键时返回默认值，**不** exit。
// 用于后期新增的键（如 record_video / record_fps）：已有 configs/*.yaml 里没有这些键，
// 用 read() 会直接 exit(1)；用这些函数则保持老配置可用（AGENTS.md §4.1.2 的新增键约定）。
inline bool optional_bool(const YAML::Node & yaml, const std::string & key, bool fallback)
{
  return yaml[key] ? yaml[key].as<bool>() : fallback;
}

inline double optional_double(const YAML::Node & yaml, const std::string & key, double fallback)
{
  return yaml[key] ? yaml[key].as<double>() : fallback;
}

inline std::string optional_string(
  const YAML::Node & yaml, const std::string & key, const std::string & fallback)
{
  return yaml[key] ? yaml[key].as<std::string>() : fallback;
}

}  // namespace tools

#endif  // TOOLS__YAML_HPP