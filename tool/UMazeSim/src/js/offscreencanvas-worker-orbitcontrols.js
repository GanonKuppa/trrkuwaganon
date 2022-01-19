'use strict';

/* global importScripts, init, THREE */
import * as THREE from './lib/three.module.js';
import * as SOrbitControls from './threejs-view.js';
import * as umobj from "./UMouseObject.js";

let scene;

function noop() {
}

class ElementProxyReceiver extends THREE.EventDispatcher {
  constructor() {
    super();
  }
  get clientWidth() {
    return this.width;
  }
  get clientHeight() {
    return this.height;
  }
  getBoundingClientRect() {
    return {
      left: this.left,
      top: this.top,
      width: this.width,
      height: this.height,
      right: this.left + this.width,
      bottom: this.top + this.height,
    };
  }
  handleEvent(data) {
    if (data.type === 'size') {
      this.left = data.left;
      this.top = data.top;
      this.width = data.width;
      this.height = data.height;
      return;
    }
    data.preventDefault = noop;
    data.stopPropagation = noop;
    this.dispatchEvent(data);
  }
  focus() {
    // no-op
  }
}

class ProxyManager {
  constructor() {
    this.targets = {};
    this.handleEvent = this.handleEvent.bind(this);
  }
  makeProxy(data) {
    const {id} = data;
    const proxy = new ElementProxyReceiver();
    this.targets[id] = proxy;
  }
  getProxy(id) {
    return this.targets[id];
  }
  handleEvent(data) {
    this.targets[data.id].handleEvent(data.data);
  }
}

const proxyManager = new ProxyManager();

function start(data) {  
  const proxy = proxyManager.getProxy(data.canvasId);
  proxy.body = proxy;  // HACK!
  self.window = proxy;
  self.document = {
    addEventListener: proxy.addEventListener.bind(proxy),
    removeEventListener: proxy.removeEventListener.bind(proxy),
  };
  scene = SOrbitControls.init({
    canvas: data.canvas,
    inputElement: proxy,
    postMessageFunc: self.postMessage
  });  
}

function makeProxy(data) {
  proxyManager.makeProxy(data);
}

function doJsonCmd(data) {
  
  try{
    let cmd = data["cmd"];
    switch (cmd){
      case 'ADD_NEEDLE':
        umobj.addNeedle(scene, data["x"], data["y"], data["name"]);
        break;
      case 'ADD_MSG_FLAG':
        umobj.addMsgFlag(scene, data["x"], data["y"], data["msg"]);
        break;
      case 'REMOVE_CONTRAIL':
        umobj.removeContrail(scene);
        break;
      case 'REMOVE_ALL_NAMED_OBJECT':
        umobj.removeAllNamedObject(scene);
        break;
      case 'REMOVE_NAMED_OBJECT':
        umobj.removeNamedObject(scene , data["name"]);
        break;
      case 'SET_WALL_32':
        umobj.setWall32(data["x"], data["y"], data["east"], data["north"], data["west"], data["south"]);
        break;
      case 'SET_ROBOT_COLOR':
        umobj.setRobotColor(data["r"], data["g"], data["b"]);        
        break;
      case 'SET_ROBOT_POS':
        umobj.setRobotPos(data["x"], data["y"], data["ang"]);
        break;
      case 'SET_TARGET_POS':
        break;
      case 'SET_WALLS_WITHOUT_OUTER_32':
        if(!data["transparent_v_mask_hex"]) data["transparent_v_mask_hex"] = "00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000";
        if(!data["transparent_h_mask_hex"]) data["transparent_h_mask_hex"] = "00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000";        
        umobj.setWallsWithoutOuter32(data["walls_v_hex"], data["walls_h_hex"], data["transparent_v_mask_hex"], data["transparent_h_mask_hex"]);        
        break;
      case 'SET_TEXT_SQUARE':
        umobj.setTextSquare(data["x"], data["y"], data["text"], data["r"], data["g"], data["b"]);
        break;
      case 'SET_ALL_TEXT_SQUARE_VISIBLE':
        umobj.setAllTextSquareVisible(data["visible"]);
        break;  
      case 'UPDATE_DATA_VIEW':
        break;
      case 'ADD_POINT_ROBOT_CONTRAIL':
        umobj.addPointRobotContrail(data["x"], data["y"], data["v"]);
        break;
      case 'ADD_POINT_TARGET_CONTRAIL':
        umobj.addPointTargetContrail(data["x"], data["y"], data["v"]);
        break;  
      case 'UPDATE_WALL_SENSOR_VIEW':
        break;
      case 'SAVE_WALLS_WITHOUT_OUTER_32':
        break;
      case 'SAVE_STL':
        break;
      default:
        break;
    }
  
  } catch (error) {
    console.log(error);
    console.log(data)
  }
  


}


const handlers = {
  start,
  makeProxy,
  event: proxyManager.handleEvent,
  doJsonCmd
};

self.onmessage = function(e) {
  const fn = handlers[e.data.type];
  if (!fn) {
    throw new Error('no handler for type: ' + e.data.type);
  }
  fn(e.data);
};