import * as THREE from './lib/three.module.js';
import { STLLoader } from './lib/STLLoader.js';
import { STLExporter } from './lib/STLExporter.js'
import * as byteUtil from  './byteUtil.js';


let ground_ref;
let table_ref;
let robot_ref;
let named_object_list = {};

// 壁の情報を格納する配列 ダブルバッファになっている
let walls32_h;
let walls32_v;
let walls32_h_buf;
let walls32_v_buf;

// 壁関係のmaterialをあらかじめ生成しておく
let mesh_walls_red = new THREE.Mesh();
let mesh_walls_red_trans = new THREE.Mesh();
let mesh_walls_white = new THREE.Mesh();
let mesh_walls_white_trans = new THREE.Mesh();

const material_walls_red = new THREE.MeshPhongMaterial({
  color: 0xff0000,
  specular: 0x666666,
});

const material_walls_white = new THREE.MeshPhongMaterial({
  color: 0xffffff,
  specular: 0x666666,
});

const material_walls_red_trans = new THREE.MeshPhongMaterial({
  color: 0xff0000,
  specular: 0x666666,
  transparent: true,
  opacity: 0.3
});

const material_walls_white_trans = new THREE.MeshPhongMaterial({
  color: 0xffffff,
  specular: 0x666666,
  transparent: true,
  opacity: 0.3
});

const geometry_wall32_v = new THREE.Mesh(new THREE.BoxGeometry(0.168 / 2, 0.05 / 2, 0.012 / 2));
const geometry_wall32r_v = new THREE.Mesh(new THREE.BoxGeometry(0.168 / 2, 0.001, 0.012 / 2));
const geometry_wall32_h = new THREE.Mesh(new THREE.BoxGeometry(0.012 / 2, 0.05 / 2, 0.168 / 2));
const geometry_wall32r_h = new THREE.Mesh(new THREE.BoxGeometry(0.012 / 2, 0.001, 0.168 / 2));

let geometry_walls32_white = new THREE.Geometry();
let geometry_walls32_white_trans = new THREE.Geometry();
let geometry_walls32_red = new THREE.Geometry();
let geometry_walls32_red_trans = new THREE.Geometry();

const material_robot_pos_contrail =  new THREE.MeshBasicMaterial({color: '#ff0000'});
const material_robot_velo_contrail =  new THREE.MeshBasicMaterial({color: '#ffff00'});

let geometry_robot_pos_contrail = new THREE.Geometry();
let geometry_robot_velo_contrail = new THREE.Geometry();
const geometry_robot_pos_cube = new THREE.Mesh(new THREE.BoxGeometry(0.003, 0.003, 0.003));
const geometry_robot_velo_cube = new THREE.Mesh(new THREE.BoxGeometry(0.003, 0.003, 0.003));


const material_target_pos_contrail =  new THREE.MeshBasicMaterial({color: '#0000ff'});
const material_target_velo_contrail =  new THREE.MeshBasicMaterial({color: '#0088ff'});

let geometry_target_pos_contrail = new THREE.Geometry();
let geometry_target_velo_contrail = new THREE.Geometry();
const geometry_target_pos_cube = new THREE.Mesh(new THREE.BoxGeometry(0.003, 0.003, 0.003));
const geometry_target_velo_cube = new THREE.Mesh(new THREE.BoxGeometry(0.003, 0.003, 0.003));

let robot_pos_contrails_ref = [];
let robot_velo_contrails_ref = [];
let target_pos_contrails_ref = [];
let target_velo_contrails_ref = [];

let textSquares32;

// 地面の追加
function addGround(scene) {
  let geometry = new THREE.PlaneBufferGeometry(0.18 * 500, 0.18 * 500, 100, 100);
  ground_ref = new THREE.Mesh(
    geometry,
    new THREE.MeshPhongMaterial({
      color: 0xaaccaa
    })
    //wireframe = true
  );
  ground_ref.rotation.x = Math.PI / -2;
  ground_ref.position.set(0, -0.25, 0)
  scene.add(ground_ref);  
}

// ライトの追加
function addLight(scene) {
  //環境光オブジェクト(light2)の設定　
  let light0 = new THREE.AmbientLight(0x333333);
  //sceneに環境光オブジェクト(light2)を追加
  scene.add(light0);

  let light1 = new THREE.SpotLight(0xffffff, 0.2, 0); // 光源の色, 強さ
  light1.position.set(-1 * (-8), 8, -8) // 光源の位置
  light1.castShadow = true;
  scene.add(light1);

  let light2 = new THREE.SpotLight(0xffffff, 0.2, 0); // 光源の色, 強さ
  light2.position.set(-1 * 8, 8, 8) // 光源の位置
  light2.castShadow = true;
  scene.add(light2);

  let light3 = new THREE.SpotLight(0xffffff, 0.2, 0); // 光源の色, 強さ
  light3.position.set(-1 * 8, 8, -8) // 光源の位置
  light3.castShadow = true;
  scene.add(light3);

  let light4 = new THREE.SpotLight(0xffffff, 0.2, 0); // 光源の色, 強さ
  light4.position.set(-1 * (-8), 8, 8) // 光源の位置
  light4.castShadow = true;
  scene.add(light4);
}

// 迷路のテーブルの追加
function addTable(scene) {
  table_ref = new THREE.Mesh(
      new THREE.BoxBufferGeometry(0.18 * 18, 0.25, 0.18 * 18),
      new THREE.MeshLambertMaterial({
        color: 0x333333
      })
  );
  table_ref.position.set(0.18 * 8.0, -0.25 / 2, 0.18 * 8.0);
  scene.add(table_ref);
}

// 柱追加
function addPillar32(scene) {
  let geometrys_white = new THREE.Geometry();
  let geometrys_red = new THREE.Geometry();

  for (let j = 0; j < 33; j++) {
    for (let i = 0; i < 33; i++) {

      let cube1 = new THREE.Mesh(new THREE.BoxGeometry(0.006, 0.025, 0.006));
      cube1.position.set(j * 0.09, 0.025 * 0.5, i * 0.09);

      let cube2 = new THREE.Mesh(new THREE.BoxGeometry(0.006, 0.0005, 0.006));
      cube2.position.set(j * 0.09, 0.025, i * 0.09);

      geometrys_white.mergeMesh(cube1);
      geometrys_red.mergeMesh(cube2);
    }

  }
  let material_pillar_white = new THREE.MeshLambertMaterial({
    color: 0xeeeeee
  });
  let mesh_white = new THREE.Mesh(geometrys_white, material_pillar_white);
  scene.add(mesh_white);


  let material_pillar_red = new THREE.MeshLambertMaterial({
    color: 0xaa3333
  });
  let mesh_red = new THREE.Mesh(geometrys_red, material_pillar_red);
  scene.add(mesh_red);
}


// 白線追加
function addWhiteLine32(scene) {


  let geometrys_white = new THREE.Geometry();
  for (let i = 0; i < 65; i++) {
    let width = 0.0005 + (0.0005 * (i % 2) );
    let cube1 = new THREE.Mesh(new THREE.BoxGeometry(width, 0.0005, 0.09*32));
    let cube2 = new THREE.Mesh(new THREE.BoxGeometry(0.09*32, 0.0005, width));    
    cube1.position.set(i * 0.09/2, 0.0, 0.09*16);
    cube2.position.set(0.09*16, 0.0, i * 0.09/2);
    geometrys_white.mergeMesh(cube1);
    geometrys_white.mergeMesh(cube2);  
  }
  
  let material_white = new THREE.MeshLambertMaterial({
    color: 0xffffff
  });
  let mesh_white = new THREE.Mesh(geometrys_white, material_white);
  scene.add(mesh_white);
}


// 迷路一区画の大きさの文字列を追加
function drawLetter32(scene, text, bg_color, text_color, text_size) {
  let texture_size = {
    width: 128,
    height: 128
  };

  function texture() {
    let canvas = new OffscreenCanvas(128, 128);//document.createElement('canvas');
    canvas.width = texture_size.width;
    canvas.height = texture_size.height;

    let context = canvas.getContext('2d');

    context.fillStyle = bg_color; //'rgba(0, 0, 255, 0.25)';
    context.fillRect(0, 0, texture_size.width, texture_size.height);
    context.font = text_size + " 'Hiragino Kaku Gothic ProN' ,'メイリオ', sans-serif";
    context.textAlign = 'center';
    context.fillStyle = text_color; //"#ffffff";
    context.fillText(text, canvas.width * 0.5, canvas.height * 0.82);

    let texture = new THREE.Texture(canvas);
    texture.needsUpdate = true;
    return texture;
  }

  let material = new THREE.MeshBasicMaterial({
    map: texture(),
    side: THREE.DoubleSide,
    transparent: true
  });

  let geometry = new THREE.PlaneGeometry(texture_size.width, texture_size.height);

  let mesh = new THREE.Mesh(geometry, material);
  mesh.scale.set(0.0009 / 2, 0.0009 / 2, 0.0009 / 2);
  scene.add(mesh);
  return mesh;
}


// 迷路のX方向区画番号を追加
function addLetterCoorX32(scene) {
  for (let i = 0; i < 32; i++) {
    let letter = drawLetter32(scene, ("0" + i).slice(-2), 'rgba(0, 0, 255, 0.25)', "#ffffff", "110px");
    letter.position.set(-0.09, 0.02, 0.045 + 0.09 * i);
    letter.rotation.set(-Math.PI / 2.0, 0, -Math.PI / 2.0);
  }
}


// 迷路のY方向区画番号を追加
function addLetterCoorY32(scene) {
  for (let i = 0; i < 32; i++) {
    let letter = drawLetter32(scene, ("0" + i).slice(-2), 'rgba(0, 0, 255, 0.25)', "#ffffff", "110px");
    letter.position.set(0.045 + 0.09 * i, 0.02, -0.09);
    letter.rotation.set(-Math.PI / 2.0, 0, -Math.PI / 2.0);
  }
}

// 区画に表示する正方形を追加
function addTextSquare32(scene){
  textSquares32 =  new Array(32);
  for (let i = 0; i < 32; i++) {
    textSquares32[i] = new Array(32);
  }

  for (let j=0; j<32; j++){
    for (let i=0; i<32; i++) {
      let letter = drawLetter32(scene, ("00000" + i), 'rgba(0, 255, 255, 0.15)', "#ffffff", "30px");
      letter.position.set(0.045 + 0.09 * i, 0.01, 0.045 + 0.09 * j);
      letter.rotation.set(-Math.PI / 2.0, 0, -Math.PI / 2.0);
      letter.visible = false;
      textSquares32[j][i] = letter;
    }
  }
}




let walls_diff = [];
function updateWalls32(){
  let walls_diff_num_pre = walls_diff.length;
  walls_diff = [];
  for(let i = 0; i< 33; i++){
    for(let j = 0; j < 33; j++){
      if(walls32_v[i][j] != walls32_v_buf[i][j])walls_diff.push({"dir":"v", "i":i,"j":j,"val": walls32_v_buf[i][j]});      
      if(walls32_h[i][j] != walls32_h_buf[i][j])walls_diff.push({"dir":"h", "i":i,"j":j,"val": walls32_h_buf[i][j]});      
    }
  }  
  if(walls_diff.length == 0) return;
  console.log(walls32_v);
  console.log(walls32_v_buf);
  if(walls_diff_num_pre == 0 && walls_diff.length > 0){
    geometry_walls32_white = new THREE.Geometry();
    geometry_walls32_white_trans = new THREE.Geometry();
    geometry_walls32_red = new THREE.Geometry();
    geometry_walls32_red_trans = new THREE.Geometry();      
  }
    
  const startTime = Date.now();
  while(walls_diff.length > 0){
    const endTime = Date.now();    
    if(endTime - startTime > 10) return;
    
    const diff = walls_diff.pop();
    const dir = diff["dir"];
    const i = diff["i"];
    const j = diff["j"];
    const val = diff["val"];
    if(dir == "v"){
      walls32_v[i][j] = walls32_v_buf[i][j];
      if(val == 0){
        
      }
      else if(val == 1){
        geometry_wall32_v.position.set((0.045 + j * 0.09), 0.025 / 2, i * 0.09);
        geometry_walls32_white.mergeMesh(geometry_wall32_v);        
        geometry_wall32r_v.position.set((0.045 + j * 0.09), 0.05 / 2, i * 0.09);
        geometry_walls32_red.mergeMesh(geometry_wall32r_v);

      }
      else if(val == 2){
        geometry_wall32_v.position.set((0.045 + j * 0.09), 0.025 / 2, i * 0.09);
        geometry_walls32_white_trans.mergeMesh(geometry_wall32_v);
        geometry_wall32r_v.position.set((0.045 + j * 0.09), 0.05 / 2, i * 0.09);
        geometry_walls32_red_trans.mergeMesh(geometry_wall32r_v);        
      }

    }
    else{
      walls32_h[i][j] = walls32_h_buf[i][j];
      if(val == 0){

      }
      else if(val == 1){
        geometry_wall32_h.position.set(i * 0.18 / 2, 0.025 / 2, 0.09 / 2 + j * 0.18 / 2);
        geometry_walls32_white.mergeMesh(geometry_wall32_h);
        geometry_wall32r_h.position.set(i * 0.18 / 2, 0.05 / 2, 0.09 / 2 + j * 0.18 / 2);
        geometry_walls32_red.mergeMesh(geometry_wall32r_h);

      }
      else if(val == 2){
        geometry_wall32_h.position.set(i * 0.18 / 2, 0.025 / 2, 0.09 / 2 + j * 0.18 / 2);
        geometry_walls32_white_trans.mergeMesh(geometry_wall32_h);
        geometry_wall32r_h.position.set(i * 0.18 / 2, 0.05 / 2, 0.09 / 2 + j * 0.18 / 2);
        geometry_walls32_red_trans.mergeMesh(geometry_wall32r_h);

      }
    }

  }

  mesh_walls_white.geometry = geometry_walls32_white;
  mesh_walls_white_trans.geometry = geometry_walls32_white_trans;
  mesh_walls_red.geometry = geometry_walls32_red;
  mesh_walls_red.geometry_trans = geometry_walls32_red_trans;
}

function wallsChangeTest32(){
  for (let i = 0; i < 33; i++) {
    walls32_v_buf[i] = new Array(32);
    walls32_h_buf[i] = new Array(32);
  }
  for (let i = 0; i < 33; i++){
    for (let j = 0; j < 32; j++){
      let num = Math.floor( Math.random() * 3 );
      walls32_v_buf[i][j] = num;
      num = Math.floor( Math.random() * 3 );
      walls32_h_buf[i][j] = num;
    }
  }
}

function addWalls32(scene){
  walls32_v = new Array(33);
  walls32_h = new Array(33);  
  walls32_v_buf = new Array(33);
  walls32_h_buf = new Array(33);


  for (let i = 0; i < 33; i++) {
    walls32_v[i] = new Array(32);
    walls32_h[i] = new Array(32);
    walls32_v_buf[i] = new Array(32);
    walls32_h_buf[i] = new Array(32);

  }
  for (let i = 0; i < 33; i++){
    for (let j = 0; j < 32; j++){
      walls32_v[i][j] = 0;
      walls32_h[i][j] = 0;
      walls32_v_buf[i][j] = 0;
      walls32_h_buf[i][j] = 0;
    }
  }
  setOuterWall32();

  scene.add(mesh_walls_white);
  scene.add(mesh_walls_white_trans);
  scene.add(mesh_walls_red);
  scene.add(mesh_walls_red_trans);

  mesh_walls_white.material = material_walls_white;
  mesh_walls_white_trans.material = material_walls_white_trans;
  mesh_walls_red.material = material_walls_red;
  mesh_walls_red_trans.material = material_walls_red_trans;
}

function setTextSquareVisible(x, y, visible){
  textSquares32[x][y].visible = visible;
}

function setAllTextSquareVisible(visible){
  for(let i=0;i<32;i++){
    for(let j=0;j<32;j++){
      textSquares32[i][j].visible = visible;
    }
  }
}


function setTextSquare(x, y, text, r, g, b){
  textSquares32[x][y].visible = true;
  const texture_size = {
    width: 128,
    height: 128
  };
  
  const text_color = "#ffffff";
  const text_size = "30px";
  const bg_color = 'rgba(' + r + ',' + g + ',' + b + ', 0.25)';
  function texture() {
    const canvas = new OffscreenCanvas(128, 128);//document.createElement('canvas');
    canvas.width = texture_size.width;
    canvas.height = texture_size.height;

    const context = canvas.getContext('2d');    

    context.fillStyle = bg_color; //'rgba(0, 0, 255, 0.25)';
    context.fillRect(0, 0, texture_size.width, texture_size.height);
    context.font = text_size + " 'Hiragino Kaku Gothic ProN' ,'メイリオ', sans-serif";
    context.textAlign = 'center';
    context.fillStyle = text_color; //"#ffffff";
    context.fillText(text, canvas.width * 0.5, canvas.height * 0.82);

    const texture = new THREE.Texture(canvas);
    texture.needsUpdate = true;
    return texture;
  }
  //drawLetter32(scene, ("00000" + i), 'rgba(0, 255, 255, 0.15)', "#ffffff", "30px");

  const material = new THREE.MeshBasicMaterial({
    map: texture(),
    side: THREE.DoubleSide,
    transparent: true
  });

  textSquares32[x][y].material = material;  

}


function setOuterWall32(){
  for (let j = 0; j < 32; j++){
      walls32_v_buf[0][j] = 1;
      walls32_h_buf[0][j] = 1;
      walls32_v_buf[32][j] = 1;
      walls32_h_buf[32][j] = 1;
  }
}


function setWall32(x, y, east, north, west, south){  
  for (let i = 0; i < 33; i++){
    for (let j = 0; j < 32; j++){
      walls32_v[i][j] = 0;
      walls32_h[i][j] = 0;
    }
  }
  
  if(walls32_v_buf[x+1][y] != west){
    walls32_v_buf[x+1][y] = west;
  }
  if(walls32_v_buf[x][y] != east){
    walls32_v_buf[x][y] = east;
  }
  if(walls32_h_buf[y+1][x] != north){
    walls32_h_buf[y+1][x] = north;
  }
  if(walls32_h_buf[y][x] != south){
    walls32_h_buf[y][x] = south;    
  }
}


function setWallsWithoutOuter32(walls_v_hex, walls_h_hex, transparent_v_mask_hex, transparent_h_mask_hex){  

  const walls_vertical_list = byteUtil.hexstr2intList(walls_v_hex, 124*2);
  const walls_horizontal_list = byteUtil.hexstr2intList(walls_h_hex, 124*2);
  const transparent_vertical_mask = byteUtil.hexstr2intList(transparent_v_mask_hex, 124*2);
  const transparent_horizontal_mask = byteUtil.hexstr2intList(transparent_h_mask_hex, 124*2)

  for (let i = 0; i < 33; i++){
    for (let j = 0; j < 32; j++){
      walls32_v[i][j] = 0;
      walls32_h[i][j] = 0;
    }
  }


  for (let i = 0; i < 31; i++) {
    const byte0 = walls_vertical_list[i * 4 + 0];
    const byte1 = walls_vertical_list[i * 4 + 1];
    const byte2 = walls_vertical_list[i * 4 + 2];
    const byte3 = walls_vertical_list[i * 4 + 3];
    const bitList = byteUtil.byte4to32bit(byte0, byte1, byte2, byte3);

    const byte0_m = transparent_vertical_mask[i * 4 + 0];
    const byte1_m = transparent_vertical_mask[i * 4 + 1];
    const byte2_m = transparent_vertical_mask[i * 4 + 2];
    const byte3_m = transparent_vertical_mask[i * 4 + 3];
    const mask = byteUtil.byte4to32bit(byte0_m, byte1_m, byte2_m, byte3_m);

    for (let j = 0; j < 32; j++) {
      if(bitList[j] == 1){
        if(mask[j] == 1) walls32_v_buf[i+1][j] = 2;
        else walls32_v_buf[i+1][j] = 1;
      } 
      else walls32_v_buf[i+1][j] = 0;
    }
  }

  
  for (let i = 0; i < 31; i++) {
    const byte0 = walls_horizontal_list[i * 4 + 0];
    const byte1 = walls_horizontal_list[i * 4 + 1];
    const byte2 = walls_horizontal_list[i * 4 + 2];
    const byte3 = walls_horizontal_list[i * 4 + 3];
    const bitList = byteUtil.byte4to32bit(byte0, byte1, byte2, byte3);

    const byte0_m = transparent_horizontal_mask[i * 4 + 0];
    const byte1_m = transparent_horizontal_mask[i * 4 + 1];
    const byte2_m = transparent_horizontal_mask[i * 4 + 2];
    const byte3_m = transparent_horizontal_mask[i * 4 + 3];
    const mask = byteUtil.byte4to32bit(byte0_m, byte1_m, byte2_m, byte3_m);
    for (let j = 0; j < 32; j++) {
      if(bitList[j] == 1){
        if(mask[j] == 1) walls32_h_buf[i+1][j] = 2;
        else walls32_h_buf[i+1][j] = 1;
      } 
      else walls32_h_buf[i+1][j] = 0;
    }
  }  
  setOuterWall32();
}

function wallsSetTest32(){
  const json_cmd = {
    "cmd": "SET_WALLS_WITHOUT_OUTER_32",
    "walls_v_hex": "000000000000000000000000000000000000e000c01f00000000000000000000000000000000000000100000001070000000a0010000000000020000000e0000000000000000f00100004000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000",
    "walls_h_hex": "0000000000000000000000000000000000000000000000008003000000000000808300000080010000000100e00300000000000000000000000000000000000000000000000000000000000000000000102000000010080000100400fe000000781c0000000000000000000000000000000000000000000000000000",
    "transparent_v_mask_hex": "0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000001070000000a0010000000000020000000e0000000000000000f00100004000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000",
    "transparent_h_mask_hex": "0000000000000000000000000000000000000000000000008003000000000000808300000080010000000100e00300000000000000000000000000000000000000000000000000000000000000000000102000000010080000100400fe000000781c0000000000000000000000000000000000000000000000000000"
  }
  setWallsWithoutOuter32(json_cmd["walls_v_hex"], json_cmd["walls_h_hex"], json_cmd["transparent_v_mask_hex"], json_cmd["transparent_h_mask_hex"]);  
}




function addRobot(scene) {
  console.log("add robot");
  //ここにstlファイルを記述
  let model_load = "../../mouse.stl";
  let loader = new STLLoader();
  loader.load(model_load, function(geometry){
    let material = new THREE.MeshLambertMaterial({color: 0xffffff });
    let mesh = new THREE.Mesh(geometry, material);
    mesh.position.set(0.09 / 2, 0.0, 0.09 / 2);
    mesh.rotation.set(0, Math.PI / 2, 0);
    scene.add(mesh);
    robot_ref = mesh;
  });  

}


function setRobotPos(x, y, ang) {
  if(!robot_ref) return;

  const ang_rad = Math.PI * ang / 180.0;
  robot_ref.position.set(y, 0.0, x);
  robot_ref.rotation.set(0, ang_rad, 0);
}

function setRobotColor(r, g, b){
  if(!robot_ref) return;
  const color = r * 256 * 256  +  g * 256 + b;
  robot_ref.material.color = new THREE.Color(color);
}


let x_robot_contrail_pre = 0.0;
let y_robot_contrail_pre = 0.0;
let robot_contrail_num = 0;
let x_robot_contrail_origin = 0.0;
let y_robot_contrail_origin = 0.0;
function addPointRobotContrail(x, y, v){  
  if(robot_contrail_num == 1){
    x_robot_contrail_origin = x;
    y_robot_contrail_origin = y;
    robot_pos_contrails_ref[robot_pos_contrails_ref.length-1].position.set(y_robot_contrail_origin, 0.0, x_robot_contrail_origin);
    robot_velo_contrails_ref[robot_velo_contrails_ref.length-1].position.set(y_robot_contrail_origin, 0.0, x_robot_contrail_origin);
  }

  geometry_robot_pos_cube.position.set(y-y_robot_contrail_origin, 0.005, x-x_robot_contrail_origin);
  if(x != x_robot_contrail_pre || y != y_robot_contrail_pre){
    geometry_robot_pos_contrail.mergeMesh(geometry_robot_pos_cube);
  }
  geometry_robot_pos_contrail.dynamic = true;
  geometry_robot_pos_contrail.elementsNeedUpdate = true;
  geometry_robot_pos_contrail.verticesNeedUpdate = true;

  geometry_robot_velo_cube.position.set(y-y_robot_contrail_origin, 0.1 + v*0.1, x-x_robot_contrail_origin);
  if(x != x_robot_contrail_pre || y != y_robot_contrail_pre){ 
    geometry_robot_velo_contrail.mergeMesh(geometry_robot_velo_cube);
  }
  geometry_robot_velo_contrail.dynamic = true;
  geometry_robot_velo_contrail.elementsNeedUpdate = true;
  geometry_robot_velo_contrail.verticesNeedUpdate = true;

  x_robot_contrail_pre = x;
  y_robot_contrail_pre = y;
  robot_contrail_num ++;
};


let x_target_contrail_pre = 0.0;
let y_target_contrail_pre = 0.0;
let target_contrail_num = 0;
function addPointTargetContrail(x, y, v){
  geometry_target_pos_cube.position.set(y, 0.005, x);
  if(x != x_target_contrail_pre || y != y_target_contrail_pre){
    geometry_target_pos_contrail.mergeMesh(geometry_target_pos_cube);
  }
  geometry_target_pos_contrail.dynamic = true;
  geometry_target_pos_contrail.elementsNeedUpdate = true;
  geometry_target_pos_contrail.verticesNeedUpdate = true;


  geometry_target_velo_cube.position.set(y, 0.1 + v, x);
  if(x != x_target_contrail_pre || y != y_target_contrail_pre){
    geometry_target_velo_contrail.mergeMesh(geometry_target_velo_cube);
  }
  geometry_target_velo_contrail.dynamic = true;
  geometry_target_velo_contrail.elementsNeedUpdate = true;
  geometry_target_velo_contrail.verticesNeedUpdate = true;

  x_target_contrail_pre = x;
  y_target_contrail_pre = y;
  target_contrail_num ++;
};


function updateRobotContrail(scene) {
  if( robot_contrail_num > 0 && robot_contrail_num < 60) return;
  robot_contrail_num = 1;
  geometry_robot_pos_contrail = new THREE.Geometry();
  geometry_robot_velo_contrail = new THREE.Geometry();
  let mesh_robot_pos_contrail_ = new THREE.Mesh(geometry_robot_pos_contrail, material_robot_pos_contrail);
  let mesh_robot_velo_contrail_ = new THREE.Mesh(geometry_robot_velo_contrail, material_robot_velo_contrail);
  scene.add(mesh_robot_pos_contrail_);
  scene.add(mesh_robot_velo_contrail_);
  robot_pos_contrails_ref.push(mesh_robot_pos_contrail_);
  robot_velo_contrails_ref.push(mesh_robot_velo_contrail_);  
}

function updateTargetContrail(scene) {
  if( target_contrail_num > 0 && target_contrail_num < 60) return;
  target_contrail_num = 1;

  geometry_target_pos_contrail = new THREE.Geometry();
  geometry_target_velo_contrail = new THREE.Geometry();
  let mesh_target_pos_contrail_ = new THREE.Mesh(geometry_target_pos_contrail, material_target_pos_contrail);
  let mesh_target_velo_contrail_ = new THREE.Mesh(geometry_target_velo_contrail, material_target_velo_contrail);

  scene.add(mesh_target_pos_contrail_);
  scene.add(mesh_target_velo_contrail_);
  target_pos_contrails_ref.push(mesh_target_pos_contrail_);
  target_velo_contrails_ref.push(mesh_target_velo_contrail_);
}

function removeContrail(scene) {  
  robot_pos_contrails_ref.forEach(function(obj){
    let geometry = obj.geometry;
    let material = obj.material;
    geometry.dispose();
    material.dispose();
    scene.remove(obj);
    robot_contrail_num = 0;
    target_contrail_num = 0;
  });

  robot_velo_contrails_ref.forEach(function(obj){
    let geometry = obj.geometry;
    let material = obj.material;
    geometry.dispose();
    material.dispose();
    scene.remove(obj);
  });

  target_pos_contrails_ref.forEach(function(obj){
    let geometry = obj.geometry;
    let material = obj.material;
    geometry.dispose();
    material.dispose();
    scene.remove(obj);
  });

  target_velo_contrails_ref.forEach(function(obj){
    let geometry = obj.geometry;
    let material = obj.material;
    geometry.dispose();
    material.dispose();
    scene.remove(obj);
  });

  robot_pos_contrails_ref = [];
  robot_velo_contrails_ref = [];
  target_pos_contrails_ref = [];
  target_velo_contrails_ref = [];
};


function addNeedle(scene, x, y, name) {
  let material_col = { color: '#888800' };

  let geometry = new THREE.BoxBufferGeometry(0.0015, 0.18, 0.0015);
  let material = new THREE.MeshBasicMaterial(material_col);

  let box = new THREE.Mesh(geometry, material);
  box.position.set(y, 0.0, x);
  named_object_list[name] = box;
  scene.add(box);
}


let msg_flag_pos_list = {};

function addMsgFlag(scene, x, y, msg, name) {
  let mesh_group = new THREE.Group();  
  let near_pos_count = 0;
  Object.keys(msg_flag_pos_list).forEach(function(key){
    if(Math.abs(msg_flag_pos_list[key].x - x) < 0.09*1.5  &&
       Math.abs(msg_flag_pos_list[key].y - y) < 0.02 ) near_pos_count ++;
  });


  let texture_size = {
    width: 256,
    height: 32
  };

  function texture() {
    let canvas_ =  new OffscreenCanvas(256, 32);//document.createElement('canvas');
    canvas_.width = texture_size.width;
    canvas_.height = texture_size.height;

    let context = canvas_.getContext('2d');

    context.fillStyle = 'rgba(255, 255, 0, 0.25)';
    context.fillRect(0, 0, texture_size.width, texture_size.height);
    context.font = " 9px 'Hiragino Kaku Gothic ProN' ,'メイリオ', sans-serif";
    context.textAlign = 'center';
    context.fillStyle = "#ffffff";
    context.fillText("("+x.toFixed(3)+","+y.toFixed(3)+") " +msg, canvas_.width * 0.5, canvas_.height * 0.5);

    let texture_ = new THREE.Texture(canvas_);
    texture_.needsUpdate = true;
    return texture_;
  }

  let material = new THREE.MeshBasicMaterial({
    map: texture(),
    side: THREE.DoubleSide,
    transparent: true
  });

  let geometry = new THREE.PlaneGeometry(texture_size.width, texture_size.height);

  let mesh = new THREE.Mesh(geometry, material);
  mesh.scale.set(0.0009 / 2, 0.0009 / 2, 0.0009 / 2);
  mesh.position.set(y, 0.2 + 0.015 * near_pos_count, x+0.05);
  mesh.rotation.set(0, -Math.PI / 2, 0);
  mesh_group.add(mesh);  

  {
    let material_col = { color: '#ffff00', transparent: true, opacity:0.25 };
    let geometry = new THREE.BoxBufferGeometry(0.0015, 0.4 + 2 * 0.015 * near_pos_count, 0.0015);
    let material = new THREE.MeshBasicMaterial(material_col);

    let box = new THREE.Mesh(geometry, material);
    box.position.set(y, 0.0, x);
    mesh_group.add(box);
  }  
  scene.add(mesh_group); 
  msg_flag_pos_list[name] = {"x": x, "y": y };
  named_object_list[name] = mesh_group;
}

function removeNamedObject (scene, name) {
  let children_num = named_object_list[name].children.length;
  if(children_num == 0){
    let geometry = named_object_list[name].geometry;
    let material = named_object_list[name].material;
    try{
      let texture =  named_object_list[name].texture;
      texture.dispose();
    }catch(e){}

    scene.remove(named_object_list[name]);
    
    geometry.dispose();
    material.dispose();
  }
  else {
    named_object_list[name].children.forEach(function(obj){
      console.log(obj);
      console.log(name);
      let geometry = obj.geometry;
      let material = obj.material;
      try{
        let texture = obj.texture;
        texture.dispose();
      }catch(e){}
      geometry.dispose();
      material.dispose();
    });
    scene.remove(named_object_list[name]);
  }
  
  if (msg_flag_pos_list[name]){
    delete msg_flag_pos_list[name];
  }
}


function removeAllNamedObject (scene) {
  Object.keys(named_object_list).forEach(function(key){
    removeNamedObject(scene, key);
  });
}


function getHexWall_v(){
  let hex_str = "";
  for(var j=0;j<31;j++){
    let byte_0 = 0;
    let byte_1 = 0;
    let byte_2 = 0;
    let byte_3 = 0;
    for(let i=0;i<8;i++){
        if(walls32_v[j+1][i+ 0] > 0) byte_0 |= ((1 << i) >>> 0);
        if(walls32_v[j+1][i+ 8] > 0) byte_1 |= ((1 << i) >>> 0);
        if(walls32_v[j+1][i+16] > 0) byte_2 |= ((1 << i) >>> 0);
        if(walls32_v[j+1][i+24] > 0) byte_3 |= ((1 << i) >>> 0);
    }
    let hex_0 = ('00' + Number(byte_0).toString(16)).slice(-2);
    let hex_1 = ('00' + Number(byte_1).toString(16)).slice(-2);
    let hex_2 = ('00' + Number(byte_2).toString(16)).slice(-2);
    let hex_3 = ('00' + Number(byte_3).toString(16)).slice(-2);
    //console.log("4byte:  " + hex_0 + hex_1 + hex_2 + hex_3);
    hex_str += (hex_0 + hex_1 + hex_2 + hex_3);

  }
  return hex_str;
}

function getHexWall_h(){
  let hex_str = "";
  for(let j=0;j<31;j++){
    let byte_0 = 0;
    let byte_1 = 0;
    let byte_2 = 0;
    let byte_3 = 0;
    for(let  i=0;i<8;i++){
        if(walls32_h[j+1][i+ 0] > 0) byte_0 |= ((1 << i) >>> 0);
        if(walls32_h[j+1][i+ 8] > 0) byte_1 |= ((1 << i) >>> 0);
        if(walls32_h[j+1][i+16] > 0) byte_2 |= ((1 << i) >>> 0);
        if(walls32_h[j+1][i+24] > 0) byte_3 |= ((1 << i) >>> 0);
    }
    let hex_0 = ('00' + Number(byte_0).toString(16)).slice(-2);
    let hex_1 = ('00' + Number(byte_1).toString(16)).slice(-2);
    let hex_2 = ('00' + Number(byte_2).toString(16)).slice(-2);
    let hex_3 = ('00' + Number(byte_3).toString(16)).slice(-2);
    //console.log("4byte:  " + hex_0 + hex_1 + hex_2 + hex_3);
    hex_str += (hex_0 + hex_1 + hex_2 + hex_3);
  }
  return hex_str;

}

function getMaze32(){  
  const data = {
    "walls_v_hex": getHexWall_v(),
    "walls_h_hex": getHexWall_h()
  }
  return data;
}


function makeMazeStlString(maze_data){
  console.log(maze_data);
  let scene = new THREE.Scene();
  let geometrys = new THREE.Geometry();
  // 土台
  let table = new THREE.Mesh(new THREE.BoxGeometry(0.18 * 18, 0.18 * 18, 0.25));
  table.position.set(0.18 * 8.0, 0.18 * 8.0, -0.25 / 2);
  geometrys.mergeMesh(table);
  
  // 柱
  for (let j = 0; j < 33; j++) {
    for (let i = 0; i < 33; i++) {

      let cube1 = new THREE.Mesh(new THREE.BoxGeometry(0.006, 0.006, 0.025));
      cube1.position.set(j * 0.09, i * 0.09, 0.025 * 0.5);

      let cube2 = new THREE.Mesh(new THREE.BoxGeometry(0.006, 0.006, 0.0005));
      cube2.position.set(j * 0.09, i * 0.09, 0.025);

      geometrys.mergeMesh(cube1);
      geometrys.mergeMesh(cube2);
    }  
  }

  // 壁外周
  let walls32_v_ = new Array(33);
  let walls32_h_ = new Array(33);  

  for (let i = 0; i < 33; i++) {
    walls32_v_[i] = new Array(32);
    walls32_h_[i] = new Array(32);
  }
  
  for (let i = 0; i < 33; i++){
    for (let j = 0; j < 32; j++){
      walls32_v_[i][j] = 0;
      walls32_h_[i][j] = 0;
    }
  }

  for (let j = 0; j < 32; j++){
    walls32_v_[0][j] = 1;
    walls32_h_[0][j] = 1;
    walls32_v_[32][j] = 1;
    walls32_h_[32][j] = 1;
  }

  // 壁中身
  const walls_vertical_list = byteUtil.hexstr2intList(maze_data["walls_v_hex"], 124*2);
  const walls_horizontal_list = byteUtil.hexstr2intList(maze_data["walls_h_hex"], 124*2);



  for (let i = 0; i < 31; i++) {
    const byte0 = walls_vertical_list[i * 4 + 0];
    const byte1 = walls_vertical_list[i * 4 + 1];
    const byte2 = walls_vertical_list[i * 4 + 2];
    const byte3 = walls_vertical_list[i * 4 + 3];
    const bitList = byteUtil.byte4to32bit(byte0, byte1, byte2, byte3);


    for (let j = 0; j < 32; j++) {
      if(bitList[j] == 1){
        walls32_v_[i+1][j] = 1;
      } 
      else walls32_v_[i+1][j] = 0;
    }
  }

  
  for (let i = 0; i < 31; i++) {
    const byte0 = walls_horizontal_list[i * 4 + 0];
    const byte1 = walls_horizontal_list[i * 4 + 1];
    const byte2 = walls_horizontal_list[i * 4 + 2];
    const byte3 = walls_horizontal_list[i * 4 + 3];
    const bitList = byteUtil.byte4to32bit(byte0, byte1, byte2, byte3);

    for (let j = 0; j < 32; j++) {
      if(bitList[j] == 1){
        walls32_h_[i+1][j] = 1;
      } 
      else walls32_h_[i+1][j] = 0;
    }
  }  
  // 壁ジオメトリセット 

  const geometry_wall32_v_ = new THREE.Mesh(new THREE.BoxGeometry(0.012 / 2, 0.168 / 2, 0.05 / 2));
  const geometry_wall32_h_ = new THREE.Mesh(new THREE.BoxGeometry(0.168 / 2, 0.012 / 2, 0.05 / 2));
  
  for (let i = 0; i < 33; i++){
    for (let j = 0; j < 32; j++){
      if(walls32_v_[i][j] == 1){
        geometry_wall32_v_.position.set((i * 0.09), j * 0.09 + 0.045, 0.025 / 2);
        geometrys.mergeMesh(geometry_wall32_v_);
      }

      if(walls32_h_[i][j] == 1){
        geometry_wall32_h_.position.set(j * 0.09 + 0.045, i * 0.09, 0.025 / 2);
        geometrys.mergeMesh(geometry_wall32_h_);
      }      
    }
  }


  // メッシュ
  let material = new THREE.MeshLambertMaterial({
    color: 0xeeeeee
  });
  let mesh_white = new THREE.Mesh(geometrys, material);
  scene.add(mesh_white);


  var exporter = new STLExporter();
  var str = exporter.parse( scene ); // Export the scene
  return str;
  

}


export{
  addGround, addLight, addTable, addPillar32, addWhiteLine32, addLetterCoorX32, addLetterCoorY32,        
  addWalls32, updateWalls32, addRobot, addNeedle, addMsgFlag, addTextSquare32,
  
  updateRobotContrail, updateTargetContrail, addPointRobotContrail, addPointTargetContrail,
  setRobotPos, setRobotColor, setWall32,
  
  removeNamedObject, removeAllNamedObject, removeContrail,
  
  setWallsWithoutOuter32, setTextSquare, setTextSquareVisible, setAllTextSquareVisible,
  
  getMaze32,
  wallsChangeTest32, wallsSetTest32,  
  makeMazeStlString
};
