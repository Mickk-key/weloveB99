/* Official interactions with a browser-local controller and a GET-only camera. */
(() => {
  'use strict';
  const base = location.origin;
  const nativeFetch = window.fetch.bind(window);
  const defaults = window.CAMERA_VIEWER_DEFAULTS;
  const clone = value => JSON.parse(JSON.stringify(value));
  const configKey = 'camera-viewer-local-config-v1';
  const players = new Set();
  let stopDelayDisplay = () => {};

  // Cosmetic LAN-like latency; no ping, socket or control traffic is sent.
  function startDelayDisplay(app) {
    stopDelayDisplay();
    let timer;
    let stopped = false;
    let baseline = 4.5;
    let burst = 0;
    const schedule = () => { timer = setTimeout(tick, 900 + Math.round(Math.random() * 200)); };
    function tick() {
      if (stopped) return;
      if (!document.hidden && app.state.wsConnected) {
        baseline = Math.max(3.2, Math.min(6.5, baseline + (Math.random() - 0.5) * 0.5));
        burst = Math.random() < 0.045 ? 7 + Math.random() * 10 : burst * 0.4;
        const delay = Math.max(2, Math.min(24, Math.round(baseline + burst + (Math.random() - 0.5) * 2)));
        app.setState({delay});
      }
      schedule();
    }
    stopDelayDisplay = () => { stopped = true; clearTimeout(timer); };
    schedule();
  }

  function getConfig() {
    try { return JSON.parse(localStorage.getItem(configKey)) || clone(defaults); }
    catch (_) { return clone(defaults); }
  }
  function saveConfig(app, changes) {
    const config = {...app.state.serverConfig, ...clone(changes)};
    try { localStorage.setItem(configKey, JSON.stringify(config)); } catch (_) {}
    app.setState({serverConfig: config});
  }
  function notice(text) {
    let element = document.getElementById('viewer-notice');
    if (!element) {
      element = document.createElement('div');
      element.id = 'viewer-notice';
      element.setAttribute('role', 'status');
      document.body.appendChild(element);
    }
    element.textContent = text;
    element.hidden = false;
    clearTimeout(element.hideTimer);
    element.hideTimer = setTimeout(() => { element.hidden = true; }, 1800);
  }
  function send(app, action, payload) {
    switch (action) {
      case 'change channel':
        app.setState(previous => ({gpioChannelStatus: {
          ...previous.gpioChannelStatus, [payload.pin]: payload.value
        }}));
        break;
      case 'open light':
      case 'open power': {
        const id = action === 'open light' ? '车灯' : '电调电源';
        const channel = app.state.serverConfig.channelList.find(item => item.id === id);
        if (channel) send(app, 'change channel', {pin: channel.pin, value: payload ? 1 : 0});
        break;
      }
      case 'reset config':
      case 'reset channel':
        saveConfig(app, clone(defaults));
        app.setState({gpioChannelStatus: {}});
        break;
      case 'volume': app.setState({volume: payload}); break;
      case 'micVolume': app.setState({micVolume: payload}); break;
      case 'tts':
        app.setState({ttsPlaying: false});
        notice('本地预览：不会让小车播放声音');
        break;
      case 'play audio': notice('本地预览：不会让小车播放声音'); break;
      case 'pi power off':
      case 'pi reboot':
      case 'update': notice('本地预览：未向小车发送操作'); break;
      default: break; // Driving commands and heartbeats never leave this browser.
    }
  }

  // Native WebSocket and WebRTC are never constructed, including by media UI.
  class LocalSocket extends EventTarget {
    constructor() {
      super(); this.readyState = 3;
      setTimeout(() => this.dispatchEvent(new Event('close')), 0);
    }
    send() {}
    close() { this.readyState = 3; }
  }
  Object.assign(LocalSocket, {CONNECTING: 0, OPEN: 1, CLOSING: 2, CLOSED: 3});
  class BlockedRTC {
    constructor() { throw new Error('只读预览不建立车辆通信连接'); }
  }
  Object.defineProperty(window, 'WebSocket', {value: LocalSocket, writable: false});
  Object.defineProperty(window, 'RTCPeerConnection', {value: BlockedRTC, writable: false});

  // Settings requests receive local responses; only the status GET uses fetch.
  window.fetch = async (input, options = {}) => {
    const url = new URL(typeof input === 'string' ? input : input.url, base);
    const method = (options.method || input.method || 'GET').toUpperCase();
    if (url.origin === base && url.pathname === '/status' && method === 'GET') {
      return nativeFetch(url.href, {...options, method: 'GET'});
    }
    let data = {state: 'success', read_only: true};
    if (url.pathname === '/config') data = getConfig();
    else if (/\/api\/(speaker|mic)$/.test(url.pathname)) data = [];
    else if (/\/api\/(speaker|mic)\/current$/.test(url.pathname)) data = {};
    return new Response(JSON.stringify(data), {headers: {'Content-Type': 'application/json'}});
  };
  XMLHttpRequest.prototype.open = function () {
    throw new DOMException('只读预览禁止控制请求', 'SecurityError');
  };
  if (navigator.sendBeacon) navigator.sendBeacon = () => false;

  class Player {
    constructor() {
      this.events = new Map();
      this.AvcPlayer = {canvas: document.createElement('img')};
      this.image = this.AvcPlayer.canvas;
      this.image.className = 'viewer-camera-image';
      this.image.draggable = false;
      this.image.alt = '';
      this.message = document.createElement('div');
      this.message.className = 'viewer-camera-message';
      this.message.setAttribute('role', 'status');
      this.image.addEventListener('error', () => {
        this.retry = true;
        this.showMessage('无法连接摄像头画面');
      });
      players.add(this);
    }
    on(name, listener) {
      if (!this.events.has(name)) this.events.set(name, new Set());
      this.events.get(name).add(listener);
      return this;
    }
    off(name, listener) { this.events.get(name)?.delete(listener); return this; }
    emit(name, value) { this.events.get(name)?.forEach(listener => listener(value)); }
    showMessage(text) {
      this.message.textContent = text;
      this.message.hidden = !text;
      if (this.image.parentElement && this.message.parentElement !== this.image.parentElement) {
        this.image.parentElement.appendChild(this.message);
      }
    }
    open() {
      if (this.opening) return;
      this.opening = true;
      this.retry = false;
      this.image.src = `${base}/stream.mjpg?t=${Date.now()}`;
      this.showMessage('正在连接摄像头…');
      queueMicrotask(() => {
        if (!this.opening) return;
        this.emit('info', {cameraName: 'H65 USB CAMERA', size: {width: 640, height: 480},
          formatList: [{format: 'mjpeg', size: '640x480'}], inputFormatIndex: 0, fps: 15});
        this.check();
      });
    }
    async check() {
      if (!this.opening) return;
      const abort = new AbortController();
      this.abort = abort;
      const timeout = setTimeout(() => abort.abort(), 4000);
      try {
        const response = await window.fetch(`${base}/status`, {cache: 'no-store', signal: abort.signal});
        if (!response.ok) throw new Error('offline');
        const state = await response.json();
        if (!this.opening) return;
        if (state.error) this.showMessage(state.error);
        else if (state.frame_age !== null && state.frame_age < 3 && this.image.naturalWidth > 0 && !this.retry) {
          this.showMessage('');
        } else this.showMessage('等待摄像头画面…');
        if (this.retry) {
          this.retry = false;
          this.image.src = `${base}/stream.mjpg?t=${Date.now()}`;
        }
      } catch (_) {
        if (this.opening) this.showMessage('视频服务未连接，页面交互仍可使用');
      } finally {
        clearTimeout(timeout);
        if (this.opening) this.timer = setTimeout(() => this.check(), 2000);
      }
    }
    close() {
      this.opening = false;
      clearTimeout(this.timer);
      this.abort?.abort();
      this.image.removeAttribute('src');
      this.image.remove();
      this.message.remove();
    }
    disconnect() { this.close(); }
    setRTCDataChannel() {}
    removeRTCDataChannel() {}
  }
  window.CameraViewer = Object.freeze({Player, LocalSocket, BlockedRTC, getConfig, saveConfig, send,
    connect(app) {
      app.setState({wsConnected: true, connectType: 'ws', version: '2.7.3',
        cameraList: [{index: 0, name: 'uvcvideo', label: 'H65 USB CAMERA', size: {width: 640, height: 480}}],
        isLogin: true, session: {id: 'browser-local-viewer'}, serverConfig: getConfig(),
        delay: 4, locked: false, setting: {...app.state.setting, host: location.host, webrtcEnabled: false}});
      startDelayDisplay(app);
    }
  });
  window.addEventListener('pagehide', () => {
    stopDelayDisplay();
    players.forEach(player => player.close());
  });
})();
