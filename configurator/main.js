// S3-OLED

const vario_curve = [
    {"vval":10,"frq":2000,"ton":100,"toff":100},
    {"vval":5,"frq":1500,"ton":200,"toff":100},
    {"vval":2.5,"frq":1250,"ton":200,"toff":150},
    {"vval":0.5,"frq":1000,"ton":200,"toff":200},
    {"vval":0,"frq":0,"ton":0,"toff":50},
    {"vval":-2.5,"frq":450,"ton":250,"toff":750},
    {"vval":-5,"frq":250,"ton":500,"toff":1000},
    {"vval":-10,"frq":200,"ton":500,"toff":1000}
];

function prepareVarioCurve(vario_curve){
    var vc = document.getElementById("vario_curve");
    vc.value = JSON.stringify(vario_curve, null, 2);
}

prepareVarioCurve(vario_curve);

function generateExportSettings(s_export=false){
    let formData = new FormData(document.getElementById('formData'));
    console.log(formData);
    
    const settings = Object.fromEntries(formData);

    document.querySelectorAll('input[type="checkbox"].setting').forEach(item => {
        // Iterates through all checkbox elements
          if (item.checked === true) {
            settings[item.id]=true;
            // Pushes checkbox value into skills array if checked
          }else{
            settings[item.id]=false;
          }
        });

    console.log(settings);
    for (const [key, value] of Object.entries(settings)) {
        if (document.getElementById(key).type == 'number' || document.getElementById(key).classList.contains('numeric')){
            settings[key] = Number(value);
        }
    }

    var vario_curve = JSON.parse(document.getElementById("vario_curve").value);
    settings["vario_curve"] = vario_curve;

    var json = [JSON.stringify(settings, null, 2)];

    if (s_export){

        var blob1 = new Blob(json, { type: "text/plain;charset=utf-8" });
    
        //Check the Browser.
        var isIE = false || !!document.documentMode;
        if (isIE) {
            window.navigator.msSaveBlob(blob1, "settings.json");
        } else {
            var url = window.URL || window.webkitURL;
            link = url.createObjectURL(blob1);
            var a = document.createElement("a");
            a.download = "settings.json";
            a.href = link;
            document.body.appendChild(a);
            a.click();
            document.body.removeChild(a);
        }
    }else{
        document.getElementById("generated-settings").style.display = "block";
        document.getElementById("generated-settings-text").value = json;
    }
}

var b_export = document.getElementById('export-settings');
b_export.addEventListener('click',()=>{
    generateExportSettings(true);
});

var b_export = document.getElementById('generate-settings');
b_export.addEventListener('click',()=>{
    generateExportSettings(false);
});

var c_settings = document.getElementById('copy-generated-settings-text');
c_settings.addEventListener('click', ()=>{
    navigator.clipboard.writeText(document.getElementById('generated-settings-text').value);
    alert("Copied to clipboard.");
});

var b_import = document.getElementById('import-settings');
b_import.addEventListener('click',()=>{
    var fr = new FileReader();
    fr.readAsText(document.getElementById('import-file').files[0]);
    fr.onload = function() {
        const json = JSON.parse(fr.result);
        console.log(json);
        for (const key in json) {

            var el = document.getElementById(key);
            console.log(key,json[key]);
            if (el){
                if (el.tagName == 'INPUT' || el.tagName == 'SELECT'){
                    if (el.type == 'checkbox'){
                        if (json[key] == true){
                            el.checked = true;
                        }else{
                            el.checked = false;
                        }
                    }else{
                        el.value = json[key];
                    }
                }
                if(el.tagName == 'TEXTAREA'){
                    el.value = JSON.stringify(json[key],null,2);
                }
            }
        }
      };
    
      fr.onerror = function() {
        console.log(fr.error);
      };
});

// Draw Vario Curve

const ctx = document.getElementById('chart');
var mychart = null;
function drawVarioChart(vario_curve){

    var varioDataset = {};

    varioDataset["frq"] = vario_curve.map(xv=>{return {"x":xv["vval"],"y":xv["frq"]}});
    varioDataset["ton"] = vario_curve.map(xv=>{return {"x":xv["vval"],"y":xv["ton"]}});
    varioDataset["toff"] = vario_curve.map(xv=>{return {"x":xv["vval"],"y":xv["toff"]}});

    mychart = new Chart(ctx, {
        type: 'line',
        options:{
          plugins: {
              dragData: {
                round: 1, // rounds the values to n decimal places 
                          // in this case 1, e.g 0.1234 => 0.1)
                showTooltip: true, // show the tooltip while dragging [default = true]
                // dragX: true // also enable dragging along the x-axis.
                               // this solely works for continous, numerical x-axis scales (no categories or dates)!
      
                // IMPORTANT - you also need to specify dragX
                dragX: true,
      
                onDragStart: function(e, element) {
                  /*
                  // e = event, element = datapoint that was dragged
                  // you may use this callback to prohibit dragging certain datapoints
                  // by returning false in this callback
                  if (element.datasetIndex === 0 && element.index === 0) {
                    // this would prohibit dragging the first datapoint in the first
                    // dataset entirely
                    return false
                  }
                  */
                },
                onDrag: function(e, datasetIndex, index, value) {         
                  /*     
                  // you may control the range in which datapoints are allowed to be
                  // dragged by returning `false` in this callback
                  if (value < 0) return false // this only allows positive values
                  if (datasetIndex === 0 && index === 0 && value > 20) return false 
                  */
                  //console.log(datasetIndex,index,value);

                  //round automatically y to integers step 10
                  mychart.data.datasets[datasetIndex].data[index].y = Math.round(mychart.data.datasets[datasetIndex].data[index].y/10)*10;


                  for (let i = 0; i < mychart.data.datasets.length; i++) {
                    if (i != datasetIndex){
                      mychart.data.datasets[i].data[index].x = value.x;
                    }
                  }
                },
                onDragEnd: function(e, datasetIndex, index, value) {
                  // you may use this callback to store the final datapoint value
                  // (after dragging) in a database, or update other UI elements that
                  // dependent on it

                  var vval = mychart.data.datasets[0].data.map( cv => cv.x);
                  var frq = mychart.data.datasets[0].data.map(cv => cv.y);
                  var ton = mychart.data.datasets[1].data.map(cv => cv.y);
                  var toff = mychart.data.datasets[2].data.map(cv => cv.y);
                  console.log("vval",vval);
                  console.log("frq",frq);
                  console.log("ton",ton);
                  console.log("toff",toff);

                  var updated_curve = vval.map( (v,i) => { 
                    return{"vval":v, "frq":frq[i], "ton":ton[i], "toff":toff[i]};
                  });

                  document.getElementById("vario_curve").value = JSON.stringify(updated_curve,null,2);
                },
              }
            },
          scales: {
              x: {
                  title: {text:"m/s",display:true},
                  type: 'linear',
                  position: 'bottom',
                  min:-10,
                  max:10
                },
              y: {
                title: {text:"Hz",display:true},
                type: 'linear',
                display: true,
                position: 'left',
                min:0,
                max:3000,
              },
              y1: {
                title: {text:"ms",display:true},
                type: 'linear',
                display: true,
                position: 'right',
                min:0,
                max:1500,
                // grid line settings
                grid: {
                  drawOnChartArea: false, // only want the grid lines for one axis to show up
                },
              },
            }
        },
        data: {
          datasets: [{
            label: 'Frequency Hz',
            data: varioDataset["frq"],
            borderWidth: 1,
            yAxisID: 'y'
          },
          {
              label: 'Sound On ms',
              data: varioDataset["ton"],
              borderWidth: 1,
              yAxisID: 'y1',
            },
            {
              label: 'Sound Off ms',
              data: varioDataset["toff"],
              borderWidth: 1,
              yAxisID: 'y1',
            }
      
          ]
        }
      });
}

function valueAt(datasetIndex, x, yax) {
    var data = mychart.data.datasets[datasetIndex].data;
    console.log(data);
    var index = data.findIndex(o => o.x <= x);
    var next = data[index - 1];
    var prev = data[index];
    console.log(prev,next);
    if (prev && next) {
      var slope = (next.y - prev.y) / (next.x - prev.x);
      return prev.y + (x - prev.x) * slope;
    }
  }

const audioCtx = new AudioContext();

function beep(type = 'square') {

    getVarioValue();
    var oscillator = audioCtx.createOscillator();
    var gainNode = audioCtx.createGain();

    oscillator.connect(gainNode);
    gainNode.connect(audioCtx.destination);

    var vlon = document.getElementById('vario_lift_on').value;
    var vson = document.getElementById('vario_sink_on').value;

    console.log(vt_el.value , vlon, vson);

    if (vt_el.value >= Number(vlon) || vt_el.value <= Number(vson)){
        gainNode.gain.value = 0.1;
    }else{
        gainNode.gain.value = 0.;
    }
    oscillator.frequency.value = vs_frequency;
    oscillator.type = type;

    setTimeout(function(){
        oscillator.start();
        setTimeout(
            function () {
                oscillator.stop();
                if (vs_playing) beep();
            },
            vs_ton
        );
    }
        ,vs_toff);
    
};

var vs_playing = false;

drawVarioChart(vario_curve);

var vs_frequency;
var vs_ton;
var vs_toff;

function getVarioValue(){
    document.getElementById("vario-test-value").innerHTML = Math.round(vt_el.value*10)/10;
    vs_frequency = valueAt(0,vt_el.value,"0");
    vs_ton = valueAt(1,vt_el.value,"1");
    vs_toff = valueAt(2,vt_el.value,"1");
    console.log(`frq: ${vs_frequency} ton:${vs_ton} toff:${vs_toff}`);
}

const vt_el = document.getElementById("vario-test");
vt_el.addEventListener('input',getVarioValue);

const test_sound = document.getElementById("play");
test_sound.addEventListener('click',function(){
    test_sound.classList.toggle("playing");
    test_sound.classList.toggle("btn-dark");
    test_sound.classList.toggle("btn-success");
    if (!test_sound.classList.contains("playing")){
        test_sound.innerHTML = "Test Sound";
        vs_playing = false;
    } else {
        test_sound.innerHTML = "Playing, press to stop.";
        vs_playing = true;
        beep();
    }
});