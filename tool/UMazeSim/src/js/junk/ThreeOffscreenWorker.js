
import * as THREE from './lib/three.module.js';
import * as umobj from "./UMouseObject.js"
//THREE.Cache.enabled = true;


onmessage = async event => {

  // メインスレッドからオフスクリーンキャンバスを受け取る
  const canvas = event.data.canvas;
  // Three.jsのライブラリの内部で style.width にアクセスされてしまう
  // 対策しないと、エラーが発生するためダミーの値を指定
  canvas.style = { width: 0, height: 0 };

  // サイズを指定
  const width = 960;
  const height = 540;


  //let stats = new Stats();
  //stats.showPanel( 0 ); // 0: fps, 1: ms, 2: mb, 3+: custom
  //document.body.appendChild( stats.dom );
  let scene = new THREE.Scene();
  let camera = new THREE.PerspectiveCamera(45, width / height, 0.1, 1000);
  camera.lookAt(new THREE.Vector3(0.0, 0, 0.0));
  camera.position.x = 0.0;
  camera.position.y = 0.0;
  camera.position.z = 10.0;
  let context = canvas.getContext( 'webgl2', { alpha: false } );
  let renderer = new THREE.WebGLRenderer( { canvas: canvas, 
                                            context: context, 
                                            antialias: true, 
                                            precision: 'highp' 
                                          } );    
  renderer.setSize(width, height);
  renderer.setPixelRatio(2);
  renderer.shadowMap.enabled = true;
  renderer.setClearColor(0xcccccc, 1.0);

  
  umobj.addGround(scene);
  umobj.addLight(scene);
  umobj.addTable(scene);
  umobj.addPillar32(scene);
  umobj.addWhiteLine32(scene);
  umobj.addLetterCoorX32(scene);
  umobj.addLetterCoorY32(scene);
  umobj.addWalls32(scene);


  animate();


  //let animate_count = 0;
  // animateループ
  function animate() {  
      //stats.begin();    
      //controls.update();
      renderer.render(scene, camera);
      //umobj.updateWalls32(scene);
      //現在のカメラの位置、角度、ターゲットをlocalStorageに保存
      //window.localStorage['camera_position'] = JSON.stringify(controls.object.position);
      //window.localStorage['camera_rotation'] = JSON.stringify(controls.object.rotation);
      //window.localStorage['camera_target'] = JSON.stringify(controls.target);    
      
      //if(animate_count % 50 == 0)umobj.wallsChangeTest32();

      //animate_count ++;
      //stats.end();
      requestAnimationFrame(animate);
  }


}







//export{animate, scene};