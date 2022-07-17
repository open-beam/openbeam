var editor = ace.edit("editor");
editor.setTheme("ace/theme/monokai");
editor.session.setMode("ace/mode/yaml");
editor.setOptions({
    enableLinking: true
});
editor.resize();

editor.on("linkClick", function (data) {
    console.log("CLICK", data); // Just for testing so you can see what classes things are...
    if (data && data.token && data.token.type == "link") {
        window.open(data.token.value, "_blank");
    }
});

// Synchronously read a text file from the web server with Ajax
//
// The filePath is relative to the web page folder.
// Example:   myStuff = loadFile("Chuuk_data.txt");
//
// You can also pass a full URL, like http://sealevel.info/Chuuk1_data.json, but there
// might be Access-Control-Allow-Origin issues. I found it works okay in Firefox, Edge,
// or Opera, and works in IE 11 if the server is configured properly, but in Chrome it only
// works if the domains exactly match (and note that "xyz.com" & "www.xyz.com" don't match).
// Otherwise Chrome reports an error:
//
//   No 'Access-Control-Allow-Origin' header is present on the requested resource. Origin 'http://sealevel.info' is therefore not allowed access.
//
// That happens even when "Access-Control-Allow-Origin *" is configured in .htaccess,
// and even though I verified the headers returned (you can use a header-checker site like
// http://www.webconfs.com/http-header-check.php to check it). I think it's a Chrome bug.
function loadFile(filePath) {
    var result = null;
    var xmlhttp = new XMLHttpRequest();
    xmlhttp.open("GET", filePath, false);
    xmlhttp.send();
    if (xmlhttp.status == 200) {
        result = xmlhttp.responseText;
    }
    return result;
}

// List of examples:
lstExamples = JSON.parse(loadFile('../examples-structures/demo-list.json'));
var selectElement = document.getElementById('demoSelect');

{
    var el = document.createElement("option");
    el.textContent = '';
    el.value = '';
    selectElement.appendChild(el);
}

for (var i = 0; i < lstExamples.length; i++) {
    var opt = lstExamples[i];
    var el = document.createElement("option");
    el.textContent = opt;
    el.value = opt;
    selectElement.appendChild(el);
}

function loadDemoFile(fil) {
    if (fil != '') {
        f = loadFile('../examples-structures/' + fil);
        editor.setValue(f, -1); // "-1": move cursor to start
    }
}

var statusElement = document.getElementById('status');
var progressElement = document.getElementById('progress');
var spinnerElement = document.getElementById('spinner');

// Instantiate app:
var obApp; // On module loaded and initialized, see below:

// This must come before "web-ob-solve.js":

var Module = {
    preRun: [],
    postRun: [],
    /*            print: (function () {
                    var element = document.getElementById('output');
                    if (element) element.value = ''; // clear browser cache
                    return function (text) {
                        if (arguments.length > 1) text = Array.prototype.slice.call(arguments).join(' ');
                        // These replacements are necessary if you render to raw HTML
                        //text = text.replace(/&/g, "&amp;");
                        //text = text.replace(/</g, "&lt;");
                        //text = text.replace(/>/g, "&gt;");
                        //text = text.replace('\n', '<br>', 'g');
                        console.log(text);
                        if (element) {
                            element.value += text + "\n";
                            element.scrollTop = element.scrollHeight; // focus on bottom
                        }
                    };
                })(),*/
    canvas: (function () {
        var canvas = document.getElementById('canvas');

        // As a default initial behavior, pop up an alert when webgl context is lost. To make your
        // application robust, you may want to override this behavior before shipping!
        // See http://www.khronos.org/registry/webgl/specs/latest/1.0/#5.15.2
        canvas.addEventListener("webglcontextlost", function (e) { alert('WebGL context lost. You will need to reload the page.'); e.preventDefault(); }, false);

        return canvas;
    })(),
    onRuntimeInitialized: function () {
        console.log('onRuntimeInitialized');
        obApp = new Module.AppOpenBeam();
        obApp.repaintCanvas();

        /*const id = setInterval(() => {
            obApp.repaintCanvas();
        }, 200);*/

    },
    setStatus: function (text) {
        if (!Module.setStatus.last) Module.setStatus.last = { time: Date.now(), text: '' };
        if (text === Module.setStatus.last.text) return;
        var m = text.match(/([^(]+)\((\d+(\.\d+)?)\/(\d+)\)/);
        var now = Date.now();
        if (m && now - Module.setStatus.last.time < 30) return; // if this is a progress update, skip it if too soon
        Module.setStatus.last.time = now;
        Module.setStatus.last.text = text;
        if (m) {
            text = m[1];
            progressElement.value = parseInt(m[2]) * 100;
            progressElement.max = parseInt(m[4]) * 100;
            progressElement.hidden = false;
            spinnerElement.hidden = false;
        } else {
            progressElement.value = null;
            progressElement.max = null;
            progressElement.hidden = true;
            if (!text) spinnerElement.style.display = 'none';
        }
        statusElement.innerHTML = text;
    },
    totalDependencies: 0,
    monitorRunDependencies: function (left) {
        this.totalDependencies = Math.max(this.totalDependencies, left);
        Module.setStatus(left ? 'Preparing... (' + (this.totalDependencies - left) + '/' + this.totalDependencies + ')' : 'All downloads complete.');
    }
};

Module.setStatus('Downloading...');
window.onerror = function (event) {
    // TODO: do not warn on ok events like simulating an infinite loop or exitStatus
    Module.setStatus('Exception thrown, see JavaScript console');
    spinnerElement.style.display = 'none';
    Module.setStatus = function (text) {
        if (text) Module.printErr('[post-exception status] ' + text);
    };
};
