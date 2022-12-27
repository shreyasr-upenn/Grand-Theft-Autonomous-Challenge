const char body[] PROGMEM = R"===( 
<!DOCTYPE html>  
 <html><body>        
 <h1>
  <button type="button" onclick="trophyHit()"> Go to Trophy </button>
  <span id="trophyoutputlabel"> </span> <br>

  <button type="button" onclick="fakeHit()"> Go to Fake </button>
  <span id="fakeoutputlabel"> </span> <br>

  <button type="button" onclick="policeHit()"> Go to Police Car </button>
  <span id="policeoutputlabel"> </span> <br>

  <button type="button" onclick="wallHit()"> Follow the Wall </button>
  <span id="walloutputlabel"> </span> <br>

  <button type="button" onclick="stopHit()"> STOP </button>
  <span id="stopoutputlabel"> </span> <br>

  <form id="frm1" action="/action_page.php">
    Enter X,Y: <input type="text" name="fname" value="-5,-5"><br>
  </form> 
  <button onclick="moveHit()">Move to X,Y</button>
  <span id="moveoutputlabel"> </span> <br>

  <button type="button" onclick="moveForward()"> Move Forward </button>
  <span id="forwardoutputlabel"> </span> <br>

  <button type="button" onclick="moveBackward()"> Move Backward </button>
  <span id="backwardoutputlabel"> </span> <br>

  <button type="button" onclick="turnRight()"> Turn Right </button>
  <span id="rightoutputlabel"> </span> <br>

  <button type="button" onclick="turnLeft()"> Turn Left </button>
  <span id="leftoutputlabel"> </span> <br>
  
  </h1>
 
 </body></html> 
<script> 
  //JAVASCRIPT HERE
  function trophyHit() {
    var xhttp = new XMLHttpRequest();
    xhttp.onreadystatechange = function() {
      if (this.readyState == 4 && this.status == 200) {
        document.getElementById("trophyoutputlabel").innerHTML = this.responseText;
      }
    };
    xhttp.open("GET", "trophy", true);
    xhttp.send();
  }  

  function fakeHit() {
    var xhttp = new XMLHttpRequest();
    xhttp.onreadystatechange = function() {
      if (this.readyState == 4 && this.status == 200) {
        document.getElementById("fakeoutputlabel").innerHTML = this.responseText;
      }
    };
    xhttp.open("GET", "fake", true);
    xhttp.send();
  } 

  function policeHit() {
    var xhttp = new XMLHttpRequest();
    xhttp.onreadystatechange = function() {
      if (this.readyState == 4 && this.status == 200) {
        document.getElementById("policeoutputlabel").innerHTML = this.responseText;
      }
    };
    xhttp.open("GET", "police", true);
    xhttp.send();
  } 

  function wallHit() {
    var xhttp = new XMLHttpRequest();
    xhttp.onreadystatechange = function() {
      if (this.readyState == 4 && this.status == 200) {
        document.getElementById("walloutputlabel").innerHTML = this.responseText;
      }
    };
    xhttp.open("GET", "wall", true);
    xhttp.send();
  } 

  function stopHit() {
    var xhttp = new XMLHttpRequest();
    xhttp.onreadystatechange = function() {
      if (this.readyState == 4 && this.status == 200) {
        document.getElementById("stopoutputlabel").innerHTML = this.responseText;
      }
    };
    xhttp.open("GET", "stop", true);
    xhttp.send();
  } 

  function moveHit() {
    var x = document.getElementById("frm1");
    var text = "";
    var i;
    for (i = 0; i < x.length ;i++) {
      text += x.elements[i].value + "<br>";
    }
    
    var xhttp = new XMLHttpRequest();
    xhttp.onreadystatechange = function() {
      if (this.readyState == 4 && this.status == 200) {
        document.getElementById("moveoutputlabel").innerHTML = this.responseText;
      }
    };
    var str = "move=";
    var res = str.concat(text);
    xhttp.open("GET", res, true);
    xhttp.send();
  } 

  function moveForward() {
    var xhttp = new XMLHttpRequest();
    xhttp.onreadystatechange = function() {
      if (this.readyState == 4 && this.status == 200) {
        document.getElementById("forwardoutputlabel").innerHTML = this.responseText;
      }
    };
    xhttp.open("GET", "forward", true);
    xhttp.send();
  } 

  function moveBackward() {
    var xhttp = new XMLHttpRequest();
    xhttp.onreadystatechange = function() {
      if (this.readyState == 4 && this.status == 200) {
        document.getElementById("backwardoutputlabel").innerHTML = this.responseText;
      }
    };
    xhttp.open("GET", "backward", true);
    xhttp.send();
  } 

  function turnRight() {
    var xhttp = new XMLHttpRequest();
    xhttp.onreadystatechange = function() {
      if (this.readyState == 4 && this.status == 200) {
        document.getElementById("rightoutputlabel").innerHTML = this.responseText;
      }
    };
    xhttp.open("GET", "right", true);
    xhttp.send();
  } 

  function turnLeft() {
    var xhttp = new XMLHttpRequest();
    xhttp.onreadystatechange = function() {
      if (this.readyState == 4 && this.status == 200) {
        document.getElementById("leftoutputlabel").innerHTML = this.responseText;
      }
    };
    xhttp.open("GET", "left", true);
    xhttp.send();
  } 
</script>
</html>
)===";
