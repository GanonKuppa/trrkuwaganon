import * as THREE from './lib/three.module.js';

// メインスレッドから通達があったとき
onmessage = async event => {
// メインスレッドからオフスクリーンキャンバスを受け取る
const canvas = event.data.canvas;
// Three.jsのライブラリの内部で style.width にアクセスされてしまう
// 対策しないと、エラーが発生するためダミーの値を指定
canvas.style = { width: 0, height: 0 };

// サイズを指定
const width = 960;
const height = 540;

// レンダラーを作成
const renderer = new THREE.WebGLRenderer({ canvas });
renderer.setSize(width, height);

// シーンを作成
const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(45, width / height);
camera.position.set(0, 0, +1000);
const directionalLight = new THREE.DirectionalLight(0xffffff);
directionalLight.position.set(0, 1, 1);
scene.add(directionalLight);


// マテリアルを作成
const material = new THREE.MeshPhongMaterial({
    color: 0xff0000,
    specular: 0x666666,
  });
const geometry = new THREE.SphereGeometry(300, 50, 50);
const mesh = new THREE.Mesh(geometry, material);
scene.add(mesh);

tick();

// 毎フレーム時に実行されるループイベントです
function tick() {
    mesh.rotation.y += 0.01;

    // レンダリング
    renderer.render(scene, camera);
    requestAnimationFrame(tick);
}
};