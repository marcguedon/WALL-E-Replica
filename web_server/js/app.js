// TODO Développer le code qui permet d'obtenir l'état du robot à l'ouverture de l'interface sur une page web

// ROS client initialization
var ros = new ROSLIB.Ros({
  url: "ws://" + window.location.hostname + ":9090/",
});

ros.on("connection", function () {
  console.log("Connected to ROSBridge");
});

ros.on("error", function (error) {
  console.error("Connection error to ROSBridge: ", error);
});

ros.on("close", function () {
  console.log("Disconnected from ROSBridge");
});

var automatic_mode_on = false;
var light_on = false;
var initial_x = null;
var initial_y = null;
var camera_dragged = false;
var joystick_dragged = false;

// Web page initialization
document.addEventListener("DOMContentLoaded", function () {
  const checkbox = document.getElementById("autoCheckbox");
  const movement_stick = document.getElementById("movementStick");
  const arm_sliders = Array.from(document.getElementsByClassName("armSlider"));
  const light_button = document.getElementById("lightButton");

  // // Activated mode recovery/display
  // fetch('/getOperatingMode')
  //     .then(function(response) {
  //         return response.json();
  //     })
  //     .then(function(data) {
  //         // If auto mode activated, manual buttons disable
  //         if (data.auto_mode) {
  //             remove_camera_drag_active_class();
  //             checkbox.checked = true;
  //             light_button.disabled = true;
  //             movement_stick.add('disabled');

  //             arm_sliders.forEach(function(slider) {
  //                 slider.disabled = true;
  //                 slider.value = 0;
  //             });

  //             console.log('WALL-E is in auto mode.');
  //         }

  //         // If manual mode activated, manual buttons activation
  //         else {
  //             set_camera_drag_active_class();
  //             checkbox.checked = false;
  //             light_button.disabled = false;
  //             movement_stick.classList.remove('disabled');

  //             arm_sliders.forEach(function(slider) {
  //                 slider.disabled = false;
  //             });

  //             console.log('WALL-E is in manual mode.');
  //         }
  //     });

  // // Light state recovery
  // fetch('/getLightState')
  //     .then(function(response) {
  //         return response.json();
  //     })
  //     .then(function(data) {
  //         if(data.light_on) light_button.style.backgroundImage = 'url(\'images/bulbOn2.png\')';
  //         else light_button.style.backgroundImage = 'url(\'images/bulbOff2.png\')';
  //     });

  // Used to move WALL-E's head
  document.addEventListener("mousemove", function (event) {
    if (!camera_dragged) return;

    // Prevent page scrolling when dragging
    event.preventDefault();

    move_head(event.clientX, event.clientY);
  });

  // Used to move WALL-E's head
  document.addEventListener("touchmove", function (event) {
    if (!camera_dragged) return;

    // Prevent page scrolling when dragging
    event.preventDefault();

    move_head(event.touches[0].clientX, event.touches[0].clientY);
  });

  // Used to move WALL-E's head
  document.addEventListener("mouseup", function (event) {
    camera_dragged = false;
  });

  // Used to move WALL-E's head
  document.addEventListener("touchend", function (event) {
    camera_dragged = false;
  });

  joystick_head = movement_stick.getElementsByClassName("joystickHead")[0];
  joystick_line = movement_stick.getElementsByClassName("joystickLine")[0];

  // Used to move WALL-E
  let handle_joystick = function (clientX, clientY) {
    // Get the bounding rectangles of the movement stick and joystick head
    let rect = movement_stick.getBoundingClientRect();
    let head_rect = joystick_head.getBoundingClientRect();

    let ratio = 0.25;

    // Calculate offset based on the joystick head's dimensions and ratio
    let offset_width = head_rect.width * (ratio - 0.5);
    let offset_height = head_rect.height * (ratio - 0.5);

    // Calculate the half-width and half-height of the joystick's effective area
    let half_width = rect.width / 2 + offset_width;
    let half_height = rect.height / 2 + offset_height;

    // Calculate normalized delta values for X and Y movement
    let dx = (clientX - rect.left + offset_width) / half_width - 1;
    let dy = (clientY - rect.top + offset_height) / half_height - 1;

    // Calculate the squared radius of the movement vector
    let radius_squared = dx * dx + dy * dy;

    // If the radius squared is greater than 1, normalize the vector
    if (radius_squared > 1) {
      let radius = Math.sqrt(radius_squared);
      dx /= radius;
      dy /= radius;
    }

    // Calculate new positions for the joystick head and line
    let new_x = (dx + 1) * half_width - offset_width;
    let new_y = (dy + 1) * half_height - offset_height;

    // Update the joystick head's position and the line's end point
    joystick_head.style.left = new_x + "px";
    joystick_head.style.top = new_y + "px";
    joystick_line.setAttribute("x2", new_x + "px");
    joystick_line.setAttribute("y2", new_y + "px");

    update_movement(dx, dy);
  };

  // Used to move WALL-E
  document.addEventListener("mouseup", function (event) {
    if (!joystick_dragged) return;

    movement_stick.classList.remove("dragging");
    joystick_dragged = false;
    joystick_head.style.left = "50%";
    joystick_head.style.top = "50%";

    joystick_line.setAttribute("x2", "50%");
    joystick_line.setAttribute("y2", "50%");
    update_movement(0, 0);
  });

  // Used to move WALL-E
  document.addEventListener("touchend", function (event) {
    if (!joystick_dragged) return;

    movement_stick.classList.remove("dragging");
    joystick_dragged = false;
    joystick_head.style.left = "50%";
    joystick_head.style.top = "50%";

    joystick_line.setAttribute("x2", "50%");
    joystick_line.setAttribute("y2", "50%");
    update_movement(0, 0);
  });

  // Used to move WALL-E
  movement_stick.onmousedown = function (event) {
    event.preventDefault();

    if (event.button === 2) return;
    if (movement_stick.classList.contains("disabled")) return;

    movement_stick.classList.add("dragging");
    joystick_dragged = true;
    handle_joystick(event.clientX, event.clientY);
  };

  // Used to move WALL-E
  movement_stick.ontouchstart = function (event) {
    event.preventDefault();

    if (movement_stick.classList.contains("disabled")) return;

    movement_stick.classList.add("dragging");
    joystick_dragged = true;
    handle_joystick(
      event.targetTouches[0].clientX,
      event.targetTouches[0].clientY
    );
  };

  // Used to move WALL-E
  document.addEventListener("mousemove", function (event) {
    if (!joystick_dragged) return;

    event.preventDefault();

    handle_joystick(event.clientX, event.clientY);
  });

  // Used to move WALL-E
  document.addEventListener("touchmove", function (event) {
    if (!joystick_dragged) return;

    event.preventDefault();

    handle_joystick(
      event.targetTouches[0].clientX,
      event.targetTouches[0].clientY
    );
  });
});

var switch_automatic_mode_service = new ROSLIB.Service({
    ros: ros,
    name: "/switch_automatic_mode",
    serviceType: "wall_e_msg_srv/SwitchAutomaticMode",
  });

function switch_automatic_mode(){
    const movement_stick = document.getElementById('movementStick');
    const arm_sliders = Array.from(document.getElementsByClassName('armSlider'));
    const light_button = document.getElementById('lightButton');
    const sound_buttons = Array.from(document.getElementsByClassName('soundButton'));

    automatic_mode_on = !automatic_mode_on

    if (automatic_mode_on == true) {
        console.log(automatic_mode_on);
        remove_camera_drag_active_class();
        light_button.disabled = true;
        light_button.style.backgroundImage = 'url(\'images/bulbOff2.png\')';
        movement_stick.classList.add('disabled');
        arm_sliders.forEach(function(slider) {
            slider.disabled = true;
            slider.value = 0;
        });
        sound_buttons.forEach(function(button) {
            button.disabled = true;
        });
    } else {
        console.log(automatic_mode_on);
        set_camera_drag_active_class();
        light_button.disabled = false;
        movement_stick.classList.remove('disabled');
        arm_sliders.forEach(function(slider) {
            slider.disabled = false;
        });
        sound_buttons.forEach(function(button) {
            button.disabled = false;
        });
    }

    var request = new ROSLIB.ServiceRequest({
        automatic_mode_on: automatic_mode_on,
    });
    
    switch_automatic_mode_service.callService(request, function (result) {
        if (result.success) console.log("WALL-E switched automatic mode with success.");
        else console.log("Error while switching automatic mode.");
    });
}

var move_head_request_running = false;
var move_head_prev_dx = 0;
var move_head_prev_dy = 0;
var move_head_next_dx = 0;
var move_head_next_dy = 0;
const threshold = 5;

var move_head_service = new ROSLIB.Service({
  ros: ros,
  name: "/move_head",
  serviceType: "wall_e_msg_srv/MoveHead",
});

function move_head(current_x, current_y) {
  move_head_next_dx = current_x;
  move_head_next_dy = current_y;

  if (move_head_request_running) return;
  if (
    move_head_prev_dx === move_head_next_dx &&
    move_head_prev_dy === move_head_next_dy
  )
    return;

  move_head_request_running = true;

  move_head_prev_dx = move_head_next_dx;
  move_head_prev_dy = move_head_next_dy;

  var diff_x = initial_x - current_x;
  var diff_y = initial_y - current_y;

  var x_direction = "none";
  var y_direction = "none";

  if (Math.abs(diff_x) > threshold) x_direction = diff_x > 0 ? "right" : "left";

  if (Math.abs(diff_y) > threshold) y_direction = diff_y > 0 ? "down" : "up";

  var request = new ROSLIB.ServiceRequest({
    x_direction: parseFloat(x_direction),
    y_direction: parseFloat(y_direction),
  });

  move_head_service.callService(request, function (result) {
    if (result.success) {
      move_head_request_running = false;
      console.log("WALL-E moved the head with success.");
    } else console.log("Error while moving head.");
  });
}

var move_request_running = false;
var prev_dx = 0;
var prev_dy = 0;
var next_dx = 0;
var next_dy = 0;

var move_service = new ROSLIB.Service({
  ros: ros,
  name: "/move",
  serviceType: "wall_e_msg_srv/Move",
});

function update_movement(dx, dy) {
  // console.log("update_movement service function is called");
  next_dx = dx;
  next_dy = dy;

  // Prevent multiple requests
  if (move_request_running) return;
  if (prev_dx === next_dx && prev_dy === next_dy) return;

  move_request_running = true;

  prev_dx = next_dx;
  prev_dy = next_dy;

  var request = new ROSLIB.ServiceRequest({
    x_direction: parseFloat(next_dx),
    y_direction: parseFloat(next_dy),
  });

  move_service.callService(request, function (result) {
    if (result.success) {
      move_request_running = false;
      console.log("WALL-E moved with success.");
    } else console.log("Error while moving.");
  });
}

var move_arm_service = new ROSLIB.Service({
  ros: ros,
  name: "/move_arm",
  serviceType: "wall_e_msg_srv/MoveArm",
});

function move_arm(arm, angle) {
  if (arm == "left") angle = 180 - angle;

  var request = new ROSLIB.ServiceRequest({
    arm_id: arm,
    angle: parseInt(angle),
  });

  move_arm_service.callService(request, function (result) {
    if (result.success) console.log("WALL-E moved an arm with success.");
    else console.log("Error while moving an arm.");
  });
}

function mouse_camera_drag_active_listener(event) {
  initial_x = event.clientX;
  initial_y = event.clientY;
  camera_dragged = true;

  event.preventDefault();
}

function touch_camera_drag_active_listener(event) {
  initial_x = event.touches[0].clientX;
  initial_y = event.touches[0].clientY;
  camera_dragged = true;

  event.preventDefault();
}

function set_camera_drag_active_class() {
  cameraDisplay.classList.add("cameraDragActive");

  const cameraDragActive =
    document.getElementsByClassName("cameraDragActive")[0];

  cameraDragActive.addEventListener(
    "mousedown",
    mouse_camera_drag_active_listener
  );
  cameraDragActive.addEventListener(
    "touchstart",
    touch_camera_drag_active_listener
  );
  cameraDragActive.setAttribute("title", "DRAG TO MOVE WALL-E'S HEAD");
}

function remove_camera_drag_active_class() {
  const cameraDragActive =
    document.getElementsByClassName("cameraDragActive")[0];

  if (cameraDragActive != undefined) {
    cameraDragActive.removeEventListener(
      "mousedown",
      mouse_camera_drag_active_listener
    );
    cameraDragActive.removeEventListener(
      "touchstart",
      touch_camera_drag_active_listener
    );
    cameraDragActive.removeAttribute("title");

    cameraDisplay.classList.remove("cameraDragActive");
  }
}

/*
 * Service functions
 */

var make_sound_service = new ROSLIB.Service({
  ros: ros,
  name: "/play_sound",
  serviceType: "wall_e_msg_srv/PlaySound",
});

function play_sound(sound_number) {
  // console.log("play_sound service function is called");
  var sound = document.getElementById("sound" + sound_number);
  var progress_bar = document.getElementById("progressBar" + sound_number);
  var sound_duration = sound.duration;
  var transition_duration = sound_duration * 1000;

  progress_bar.style.transition = "width " + transition_duration + "ms linear";
  progress_bar.style.width = "100%";

  sound.volume = 0.05;
  sound.play();

  sound.addEventListener("ended", function () {
    progress_bar.style.transition = "width 0ms linear";
    progress_bar.style.width = "0%";
  });

  var request = new ROSLIB.ServiceRequest({
    sound_id: parseInt(sound_number, 10),
    duration: parseFloat(sound_duration),
  });

  make_sound_service.callService(request, function (result) {
    if (result.success) console.log("WALL-E played the sound with success.");
    else console.log("Error while sound reading.");
  });
}

var set_volume_service = new ROSLIB.Service({
    ros: ros,
    name: "/set_volume",
    serviceType: "wall_e_msg_srv/SetVolume",
});
  
function set_volume(volume) {
    // console.log("set_volume service function is called")
  
    var request = new ROSLIB.ServiceRequest({
      volume: parseInt(volume, 10),
    });
  
    set_volume_service.callService(request, function (result) {
      if (result.success)
        console.log("WALL-E modified volume with success.");
      else console.log("Error while modifying the volume.");
    });
}

var set_intensity_service = new ROSLIB.Service({
  ros: ros,
  name: "/set_intensity",
  serviceType: "wall_e_msg_srv/SetIntensity",
});

function switch_light() {
  // console.log("switch_light service function is called")
  const lightButton = document.getElementById("lightButton");
  var intensity_pct = null;
  
  light_on = !light_on;

  if (light_on == true) {
    intensity_pct = 100;
    lightButton.style.backgroundImage = "url('images/bulbOn2.png')";
  } else {
    intensity_pct = 0;
    lightButton.style.backgroundImage = "url('images/bulbOff2.png')";
  }

  var request = new ROSLIB.ServiceRequest({
    light_id: "camera_light",
    intensity_pct: parseInt(intensity_pct, 10),
  });

  set_intensity_service.callService(request, function (result) {
    if (result.success)
      console.log("WALL-E switched camera light on/off with success.");
    else console.log("Error while switching camera light on/off.");
  });
}

/*
 * Callback functions
 */

var sensors_subscriber = new ROSLIB.Topic({
  ros: ros,
  name: "/distances_topic",
  messageType: "std_msgs/msg/Float32MultiArray",
});

sensors_subscriber.subscribe(function (message) {
  // console.log("sensors_subscriber callback function is called")
  const left_sensor_value = document.getElementById("leftSensor");
  const right_sensor_value = document.getElementById("rightSensor");
  const front_sensor_value = document.getElementById("frontSensor");

  left_distance = message.data[0];
  right_distance = message.data[1];
  front_distance = message.data[2];

  if (left_distance === undefined)
    left_sensor_value.textContent = "Left sensor: no sensor found";
  if (left_distance < 2 || left_distance > 400)
    left_sensor_value.textContent = "Left sensor: no obstacle";
  else
    left_sensor_value.textContent =
      "Left sensor: " + left_distance.toFixed(2) + "cm";

  if (right_distance === undefined)
    right_sensor_value.textContent = "Right sensor: no sensor found";
  else if (right_distance < 2 || right_distance > 400)
    right_sensor_value.textContent = "Right sensor: no obstacle";
  else
    right_sensor_value.textContent =
      "Right sensor: " + right_distance.toFixed(2) + "cm";

  if (front_distance === undefined)
    front_sensor_value.textContent = "Front sensor: no sensor found";
  else if (front_distance < 2 || front_distance > 400)
    front_sensor_value.textContent = "Front sensor: no obstacle";
  else
    front_sensor_value.textContent =
      "Front sensor: " + front_distance.toFixed(2) + "cm";
});

var battery_charge_subscriber = new ROSLIB.Topic({
  ros: ros,
  name: "/battery_charge_topic",
  messageType: "std_msgs/msg/Int8",
});

battery_charge_subscriber.subscribe(function (message) {
  // console.log("battery_charge_subscriber callback function is called")
  const battery_progress = document.getElementById("batteryProgress");
  const battery_text = document.getElementById("batteryPercent");

  var battery_pct = message.data;
  var battery_height = (battery_pct * 18) / 100;

  battery_progress.style.height = battery_height + "px";
  battery_text.textContent = battery_pct + "%";

  if (battery_pct < 20) battery_progress.style.background = "red";
  else battery_progress.style.background = "green";
});

var image_topic = new ROSLIB.Topic({
  ros: ros,
  name: "/camera_frame_topic",
  messageType: "sensor_msgs/Image",
});

image_topic.subscribe(function (message) {
  // console.log("image_topic callback function is called");
  var img = document.getElementById("cameraDisplay");
  img.src = "data:image/jpeg;base64," + message.data;
});
