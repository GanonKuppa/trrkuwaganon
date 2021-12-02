/*

import * as THREE from './lib/three.module.js';
import {OrbitControls} from './lib/OrbitControls.js';
import {Stats} from './lib/stats.module.js';
import * as umobj from "./UMouseObject.js"
THREE.Cache.enabled = true;



function generateScene(){
  let scene = new THREE.Scene();
  return scene;
}

function generateCamera(){
  let camera = new THREE.PerspectiveCamera(45, window.innerWidth / window.innerHeight, 0.1, 1000);
  return camera;
}

function generateControls(camera, renderer) {
  let controls = new OrbitControls(camera, renderer.domElement);
  if (window.localStorage['camera_position'] != null &&
      window.localStorage['camera_rotation'] != null &&
      window.localStorage['camera_target'] != null) {  
    
    camera.position.x = JSON.parse(window.localStorage['camera_position']).x;
    camera.position.y = JSON.parse(window.localStorage['camera_position']).y;
    camera.position.z = JSON.parse(window.localStorage['camera_position']).z;
    camera.rotation._x = JSON.parse(window.localStorage['camera_rotation'])._x;
    camera.rotation._y = JSON.parse(window.localStorage['camera_rotation'])._y;
    camera.rotation._z = JSON.parse(window.localStorage['camera_rotation'])._z;
    controls.target.x = JSON.parse(window.localStorage['camera_target']).x;
    controls.target.y = JSON.parse(window.localStorage['camera_target']).y;
    controls.target.z = JSON.parse(window.localStorage['camera_target']).z;  
  } else {
    camera.position.z = 5;
    console.log("null");
  }
  return controls;
}


// rendererの生成
function generateRenderer() {
    let canvas = document.createElement( 'canvas' );
    let context = canvas.getContext( 'webgl2', { alpha: false } );
    let renderer = new THREE.WebGLRenderer( { canvas: canvas, 
                                              context: context, 
                                              antialias: true, 
                                              precision: 'highp' 
                                            } );    
    renderer.setSize(window.innerWidth, window.innerHeight);
    renderer.setPixelRatio(2);
    renderer.shadowMap.enabled = true;
    renderer.setClearColor(0xcccccc, 1.0);
    document.body.appendChild(renderer.domElement);
    return renderer;
  }


let scene = generateScene();
let camera = generateCamera();
let renderer = generateRenderer();
let controls = generateControls(camera, renderer);
let stats = new Stats();
stats.showPanel( 0 ); // 0: fps, 1: ms, 2: mb, 3+: custom
document.body.appendChild( stats.dom );


let animate_count = 0;
// animateループ
function animate() {  
    stats.begin();    
    controls.update();
    renderer.render(scene, camera);
    umobj.updateWalls32(scene);
    //現在のカメラの位置、角度、ターゲットをlocalStorageに保存
    window.localStorage['camera_position'] = JSON.stringify(controls.object.position);
    window.localStorage['camera_rotation'] = JSON.stringify(controls.object.rotation);
    window.localStorage['camera_target'] = JSON.stringify(controls.target);    
    
    if(animate_count % 50 == 0)umobj.wallsChangeTest32();

    animate_count ++;
    stats.end();
    requestAnimationFrame(animate);
}

export{animate, scene};

*/