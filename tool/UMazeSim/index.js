'use strict';

// Electronのモジュール
const electron = require("electron");
const dgram = require('dgram');


// アプリケーションをコントロールするモジュール
const app = electron.app;

// ウィンドウを作成するモジュール
const BrowserWindow = electron.BrowserWindow;
const Menu = electron.Menu;
const dialog = electron.dialog;
const fs = require('fs');

// メインウィンドウはGCされないようにグローバル宣言
let mainWindow;
let subWindow;

// 全てのウィンドウが閉じたら終了
app.on('window-all-closed', function() {
  if (process.platform != 'darwin') {
    app.quit();
  }
});


const yaml = require('js-yaml');
// configファイルを読み込み
try {
  const config = yaml.safeLoad(fs.readFileSync('./src/config.yaml', 'utf8'));
  var UDP_PORT = config["UDP_PORT"];
  var WINDOW_WIDTH = config["WINDOW_WIDTH"];
  var WINDOW_HEIGHT =  config["WINDOW_HEIGHT"];  
} catch (e) {
  console.log(e);
}

const template = [
  {
      label:"edit",
      submenu:[
          { label:"open mazeEditor", click:()=> {
            const udp_server = dgram.createSocket('udp4');
            const data = {
              "cmd": "SAVE_WALLS_WITHOUT_OUTER_32",
              "path": "temp_maze.json"
            };
            const send_data = Buffer.from(JSON.stringify(data));
            udp_server.send(send_data, 0, send_data.length, UDP_PORT, "127.0.0.1", (err, bytes) => {
                if (err) throw err;
            });  
          

            subWindow = new BrowserWindow({width: 750, height: 650, useContentSize: true, webPreferences: { nodeIntegration: true , zoomFactor: 0.5}});  
            subWindow.removeMenu();
            subWindow.loadURL('file://' + __dirname + '/UMazeEditor.html');
          }},
          { label:"import maze (json)", click:()=> {
            openFile();
            //subWindow = new BrowserWindow({width: 750, height: 650, useContentSize: true, webPreferences: { nodeIntegration: true , zoomFactor: 0.5}});  
            //subWindow.removeMenu();
            //subWindow.loadURL('file://' + __dirname + '/UMazeEditor.html');
          }},
          { label:"export maze (json)", click:()=> {
            saveFile();
            //subWindow = new BrowserWindow({width: 750, height: 650, useContentSize: true, webPreferences: { nodeIntegration: true , zoomFactor: 0.5}});  
            //subWindow.removeMenu();
            //subWindow.loadURL('file://' + __dirname + '/UMazeEditor.html');
          }},
          { label:"export maze (stl)", click:()=> {
            saveStl();
            //subWindow = new BrowserWindow({width: 750, height: 650, useContentSize: true, webPreferences: { nodeIntegration: true , zoomFactor: 0.5}});  
            //subWindow.removeMenu();
            //subWindow.loadURL('file://' + __dirname + '/UMazeEditor.html');
          }},

  
      ]
  },

  {
    label: 'view',
    submenu: [
      {
        label: 'Reload',
        accelerator: 'CmdOrCtrl+R',
        click: function(item, focusedWindow) {
          if (focusedWindow) focusedWindow.reload();
        }
      },
      {
        label: 'Toggle Full Screen',
        accelerator: (function() {
          if (process.platform == 'darwin')
            return 'Ctrl+Command+F';
          else
            return 'F11';
        })(),
        click: function(item, focusedWindow) {
          if (focusedWindow) focusedWindow.setFullScreen(!focusedWindow.isFullScreen());
        }
      },
      {
        label: 'Toggle Developer Tools',
        accelerator: (function() {
          if (process.platform == 'darwin')
            return 'Alt+Command+I';
          else
            return 'Ctrl+Shift+I';
        })(),
        click: function(item, focusedWindow) {
          if (focusedWindow) focusedWindow.webContents.toggleDevTools();
        }
      },
    ]
  },
];


// Electronの初期化完了後に実行
app.allowRendererProcessReuse = true; // 追加
app.on('ready', function() {  
  mainWindow = new BrowserWindow({width: WINDOW_WIDTH, height: WINDOW_HEIGHT, useContentSize: true, webPreferences: { nodeIntegration: true }});  
  //mainWindow.setMenu(null);
  mainWindow.loadURL('file://' + __dirname + '/index.html');
  const menu = Menu.buildFromTemplate(template);
  Menu.setApplicationMenu(menu);

  // ウィンドウが閉じられたらアプリも終了
  mainWindow.on('closed', function() {
    mainWindow = null;
  });
});

function saveFile() {
  const win = BrowserWindow.getFocusedWindow(); 
  
  const filepath = dialog.showOpenDialogSync(
      win,
      {
          title: "迷路データ保存",
          properties: ['promptToCreate'],
          filters: [
              {
                  name: 'Desktop',
                  extensions: ['json']
              }
          ]
      }
  );
  
  const udp_server = dgram.createSocket('udp4');
  const data = {
    "cmd": "SAVE_WALLS_WITHOUT_OUTER_32",
    "path": filepath[0]
  };
  const send_data = Buffer.from(JSON.stringify(data));
  udp_server.send(send_data, 0, send_data.length, UDP_PORT, "127.0.0.1", (err, bytes) => {
      if (err) throw err;
  });  

}


//openFileボタンが押されたとき（ファイル名取得まで）
function openFile() {  
  const win = BrowserWindow.getFocusedWindow(); 
  
  const filepath = dialog.showOpenDialogSync(
      win,
      {
          properties: ['openFile'],
          filters: [
              {
                  name: 'Desktop',
                  extensions: ['json']
              }
          ]
      }
  );
  readFile(filepath[0]);

}

//指定したファイルを読み込む
function readFile(path) {
  
  fs.readFile(path, (error, data) => {
    
    const udp_server = dgram.createSocket('udp4');
    let send_data = JSON.parse(data.toString());
    send_data["cmd"] = "SET_WALLS_WITHOUT_OUTER_32";
    let byte_data = Buffer.from(JSON.stringify(send_data));
    udp_server.send(byte_data, 0, byte_data.length, UDP_PORT, "127.0.0.1", (err, bytes) => {
        if (err) throw err;
    });  
  })
}

function saveStl() {
  const udp_server = dgram.createSocket('udp4');
  const data = {
    "cmd": "SAVE_STL"
  };
  const send_data = Buffer.from(JSON.stringify(data));
  udp_server.send(send_data, 0, send_data.length, UDP_PORT, "127.0.0.1", (err, bytes) => {
      if (err) throw err;
  });  

}