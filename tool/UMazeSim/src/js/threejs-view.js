import * as THREE from './lib/three.module.js';
import * as umobj from './UMouseObject.js'
import {OrbitControls} from './lib/OrbitControls.js';


let x = 0.045;
let y = 0.045;
let ang = 90.0;
let v = 0;

let x_target = 0.045;
let y_target = 0.045;
let v_target = 0;


function init(data) {   /* eslint-disable-line no-unused-vars */
  const {canvas, inputElement, postMessageFunc} = data;
  
  let context = canvas.getContext( 'webgl2', { alpha: false } );
  let renderer = new THREE.WebGLRenderer( { canvas: canvas, 
    context: context, 
    antialias: true, 
    precision: 'highp' 
  } );   
  renderer.setPixelRatio(2);
  renderer.shadowMap.enabled = true;
  renderer.setClearColor(0xcccccc, 1.0);


  const fov = 25;
  const aspect = 2; // the canvas default
  const near = 0.1;
  const far = 1000;
  const camera = new THREE.PerspectiveCamera(fov, aspect, near, far);
  //const controls = new THREE.OrbitControls(camera, inputElement);  // change
  camera.position.set(1.57, 5.5, 1.57);

  const controls = new OrbitControls(camera, inputElement);  // change
  controls.target.set(1.57,0.0,1.57);
  controls.rotate(1.57);
  
  const scene = new THREE.Scene();

  umobj.addGround(scene);
  umobj.addLight(scene);
  umobj.addTable(scene);
  umobj.addPillar32(scene);
  umobj.addWhiteLine32(scene);
  umobj.addLetterCoorX32(scene);
  umobj.addLetterCoorY32(scene);
  umobj.addWalls32(scene);
  umobj.addRobot(scene);
  umobj.addTextSquare32(scene);
    
  function resizeRendererToDisplaySize(renderer) {
    const canvas = renderer.domElement;
    const width = inputElement.clientWidth;
    const height = inputElement.clientHeight;
    const needResize = canvas.width !== width || canvas.height !== height;
    if (needResize) {
      renderer.setSize(width, height, false);
    }
    return needResize;
  }

  let animate_count = 0;
  let fps_begin_time = 0;
  let fps_end_time = 0;
  umobj.setRobotPos(x,y,ang);

  function render() {
    
    if(animate_count % 60 == 0) fps_begin_time = ( performance || Date ).now();    
    umobj.updateRobotContrail(scene);    
    umobj.updateTargetContrail(scene);
    
    if (resizeRendererToDisplaySize(renderer)) {
      camera.aspect = inputElement.clientWidth / inputElement.clientHeight;
      camera.updateProjectionMatrix();
    }

    renderer.render(scene, camera);
    umobj.updateWalls32();
    
    if(animate_count % 60 == 59){
      fps_end_time = ( performance || Date ).now();
      let time_diff = fps_end_time - fps_begin_time;
      let fps = 60 / (time_diff / 1000);
      const post_data =
      {
        "fps":fps,
        "maze_data":umobj.getMaze32()
      };
      postMessageFunc(post_data);

    } 
    animate_count ++;
    requestAnimationFrame(render);
    
  }

  requestAnimationFrame(render);  
  return scene;
}


export {init};