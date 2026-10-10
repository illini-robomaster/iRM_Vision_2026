'use strict';
const $ = id => document.getElementById(id);
let state = null;
let pending = false;
let initialized = false;
let sampleSignature = '';
let previewObjectUrl = null;
let previewFrames = 0;
let previewWindow = performance.now();
let previewFps = 0;

function message(text, error = false) {
  $('message').textContent = text;
  $('message').className = error ? 'error' : '';
}
async function request(path, data) {
  const response = await fetch(path, data === undefined ? {cache: 'no-store'} : {
    method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(data)
  });
  const json = await response.json();
  if (!response.ok) throw new Error(json.error || `HTTP ${response.status}`);
  return json;
}
async function action(path, data = {}) {
  if (pending) return;
  pending = true;
  render();
  try {
    await request(path, data);
    message(path === '/api/calibrate' ? '正在后台计算，请稍候…' : '操作成功');
  } catch (error) { message(error.message, true); }
  finally { pending = false; await refresh(); }
}
function render() {
  if (!state) return;
  const locked = pending || state.busy;
  $('cameraStatus').textContent = `${state.connected ? '已连接' : '未连接'} · ${state.width}×${state.height} · ${state.found ? '完整圆点板已检测' : '未检测到完整圆点板'} · 取流 ${(state.capture_fps || 0).toFixed(1)} fps · 预览 ${previewFps.toFixed(1)} fps · 检测 ${(state.detect_ms || 0).toFixed(0)} ms`;
  $('capture').disabled = locked || !state.connected || !state.found || state.samples.length >= 100;
  $('calibrate').disabled = locked || state.samples.length < 5;
  $('pattern').disabled = locked;
  $('next').hidden = !state.offline;
  $('next').disabled = locked || state.input_index + 1 >= state.input_count;
  $('download').disabled = !state.result || locked;
  $('undistort').disabled = !state.result;
  if (!state.result) $('undistort').checked = false;
  $('sampleCount').textContent = state.samples.length;
  $('directory').textContent = state.directory;
  $('progress').textContent = state.busy ? '计算中，样本修改已锁定…' : `${state.samples.length} 张有效样本。`;
  $('coverage').replaceChildren(...state.coverage.map(on => {
    const cell = document.createElement('div'); cell.className = on ? 'cell on' : 'cell'; return cell;
  }));
  $('coverageText').textContent = `中心位置覆盖 ${state.coverage.filter(Boolean).length} / 9 个区域`;
  if (!initialized) {
    $('cols').value = state.pattern.cols; $('rows').value = state.pattern.rows;
    $('spacing').value = state.pattern.spacing_mm; initialized = true;
  }
  const signature = JSON.stringify(state.samples) + locked;
  if (signature !== sampleSignature) {
    sampleSignature = signature;
    $('samples').replaceChildren(...state.samples.map(sample => {
      const card = document.createElement('div'); card.className = 'sample';
      const image = document.createElement('img'); image.src = `/sample.jpg?id=${sample.id}`;
      image.alt = `样本 ${sample.id}`; image.loading = 'lazy'; image.style.cursor = 'zoom-in';
      image.onclick = () => { $('sampleView').src = image.src; $('viewer').showModal(); };
      const label = document.createElement('div'); label.textContent = `#${sample.id}${sample.mean_px === undefined ? '' : ` · ${sample.mean_px.toFixed(3)} px`}`;
      const remove = document.createElement('button'); remove.textContent = '删除'; remove.className = 'danger'; remove.disabled = locked;
      remove.onclick = () => { if (confirm(`删除样本 #${sample.id}？`)) action('/api/delete', {id: sample.id}); };
      card.append(image, label, remove); return card;
    }));
  }
  $('rms').textContent = state.result ? `${state.result.rms_px.toFixed(4)} px` : '—';
  $('mean').textContent = state.result ? `${state.result.mean_px.toFixed(4)} px` : '—';
  if (state.result) {
    const k = state.result.camera_matrix;
    $('result').textContent = `fx=${k[0].toFixed(4)}   fy=${k[4].toFixed(4)}\ncx=${k[2].toFixed(4)}   cy=${k[5].toFixed(4)}\n\ncamera_matrix:\n${JSON.stringify(k)}\n\ndistort_coeffs:\n${JSON.stringify(state.result.distort_coeffs)}`;
  } else $('result').textContent = '尚无有效标定结果。至少 5 张，建议采集 15–25 张不同位置、距离与倾角的图片。';
  $('errors').replaceChildren(...state.samples.filter(s => s.mean_px !== undefined).map(sample => {
    const row = document.createElement('tr');
    for (const value of [sample.id, sample.mean_px.toFixed(4), sample.rms_px.toFixed(4)]) {
      const cell = document.createElement('td'); cell.textContent = value; row.append(cell);
    }
    return row;
  }));
}
async function refresh() {
  try {
    const previous = state;
    state = await request('/api/status'); render();
    if (state.error) message(state.error, true);
    else if (!previous) message('服务已连接；请准备圆点板并采集不同姿态的样本。');
    else if (previous.busy && !state.busy && state.result) message('标定完成。请检查误差和去畸变预览，再下载结果。');
  } catch (error) {
    message(`连接失败：${error.message}`, true);
    for (const id of ['capture', 'calibrate', 'pattern', 'next', 'download']) $(id).disabled = true;
  }
}
async function pollStatus() {
  await refresh();
  setTimeout(pollStatus, 500);
}
async function pollPreview() {
  const started = performance.now();
  if (state && state.connected) {
    try {
      const response = await fetch(`/preview.jpg?undistort=${$('undistort').checked ? 1 : 0}`, {cache: 'no-store'});
      if (response.ok) {
        const url = URL.createObjectURL(await response.blob());
        const decoded = new Image();
        decoded.src = url;
        try { await decoded.decode(); }
        catch (error) { URL.revokeObjectURL(url); throw error; }
        $('preview').src = url;
        if (previewObjectUrl) URL.revokeObjectURL(previewObjectUrl);
        previewObjectUrl = url;
        ++previewFrames;
        const elapsed = performance.now() - previewWindow;
        if (elapsed >= 1000) {
          previewFps = previewFrames * 1000 / elapsed;
          previewFrames = 0; previewWindow = performance.now();
        }
      }
      else message(`预览请求失败：HTTP ${response.status}`, true);
    } catch (error) { message(`预览连接失败：${error.message}`, true); }
  }
  // One preview request at a time: slow links drop refreshes, never queue stale frames.
  setTimeout(pollPreview, Math.max(0, 33 - (performance.now() - started)));
}
$('capture').onclick = () => action('/api/capture');
$('next').onclick = () => action('/api/next');
$('calibrate').onclick = () => action('/api/calibrate');
$('pattern').onclick = () => {
  const clear = state && state.samples.length > 0;
  if (clear && !confirm('修改标定板规格将删除所有当前样本，继续？')) return;
  action('/api/pattern', {cols: Number($('cols').value), rows: Number($('rows').value),
    spacing_mm: Number($('spacing').value), clear});
};
$('download').onclick = async () => {
  try {
    const response = await fetch('/intrinsics.yaml');
    if (!response.ok) throw new Error('没有可下载的有效结果');
    const url = URL.createObjectURL(await response.blob());
    const link = document.createElement('a'); link.href = url; link.download = 'intrinsics.yaml'; link.click();
    setTimeout(() => URL.revokeObjectURL(url), 1000);
  } catch (error) { message(error.message, true); }
};
$('closeViewer').onclick = () => $('viewer').close();
pollStatus();
pollPreview();