var AIOn = false;
var initialX = null;
var initialY = null;
var cameraDragged = false;
var joystickDragged = false;

//Web page initialization
document.addEventListener('DOMContentLoaded', function() {
    const checkbox = document.getElementById('autoCheckbox');
    const movementStick = document.getElementById('movementStick');
    const armSliders = Array.from(document.getElementsByClassName('armSlider'));
    const lightButton = document.getElementById('lightButton');

    //Activated mode recovery/display
    fetch('/getOperatingMode')
        .then(function(response) {
            return response.json();
        })
        .then(function(data) {
            //If auto mode activated, manual buttons disable
            if (data.auto_mode) {
                removeCameraDragActiveClass();
                checkbox.checked = true;
                lightButton.disabled = true;
                movementStick.add('disabled');

                armSliders.forEach(function(slider) {
                    slider.disabled = true;
                    slider.value = 0;
                });

                console.log('WALL-E is in auto mode.');
            }
            
            //If manual mode activated, manual buttons activation
            else {
                setCameraDragActiveClass();
                checkbox.checked = false;
                lightButton.disabled = false;
                movementStick.classList.remove('disabled');

                armSliders.forEach(function(slider) {
                    slider.disabled = false;
                });

                console.log('WALL-E is in manual mode.');
            }
        });

    //Light state recovery
    fetch('/getLightState')
        .then(function(response) {
            return response.json();
        })
        .then(function(data) {
            if(data.light_on) lightButton.style.backgroundImage = 'url(\'static/images/bulbOn2.png\')';
            else lightButton.style.backgroundImage = 'url(\'static/images/bulbOff2.png\')';
        });

    const leftSensorValue = document.getElementById('leftSensor');
    const rightSensorValue = document.getElementById('rightSensor');

    //Ultrasonic sensors recovery
    fetch('/getSensorsValue')
        .then(function(response) {
            return response.json();
        })
        .then(function(data) {
            //Sensors work between 2cm and 4m
            if(data[0] < 2 || data[0] > 400) leftSensorValue.textContent = 'Left sensor: no obstacle';
            else leftSensorValue.textContent = 'Left sensor: ' + data[0] + 'cm';

            if(data[1] < 2 || data[1] > 400) rightSensorValue.textContent = 'Right sensor: no obstacle';
            else rightSensorValue.textContent = 'Right sensor: ' + data[1] + 'cm';
        });

    //Used to move WALL-E's head
    document.addEventListener('mousemove', function(event) {
        if (!cameraDragged) return;

        //Prevent page scrolling when dragging
        event.preventDefault();

        moveHead(event.clientX, event.clientY);
    });

    //Used to move WALL-E's head
    document.addEventListener('touchmove', function(event) {
        if (!cameraDragged) return;

        //Prevent page scrolling when dragging
        event.preventDefault();

        moveHead(event.touches[0].clientX, event.touches[0].clientY);
    });
    
    //Used to move WALL-E's head
    document.addEventListener('mouseup', function(event) {
        cameraDragged = false;
    });

    //Used to move WALL-E's head
    document.addEventListener('touchend', function(event) {
        cameraDragged = false;
    });

    joystickHead = movementStick.getElementsByClassName('joystickHead')[0];
    joystickLine = movementStick.getElementsByClassName('joystickLine')[0];

    //Used to move WALL-E
    let handleJoystick = function(clientX, clientY) {
        //Get the bounding rectangles of the movement stick and joystick head
        let rect = movementStick.getBoundingClientRect();
        let headRect = joystickHead.getBoundingClientRect();

        let ratio = 0.25;

        //Calculate offset based on the joystick head's dimensions and ratio
        let offsetWidth = headRect.width * (ratio - 0.5);
        let offsetHeight = headRect.height * (ratio - 0.5);

        //Calculate the half-width and half-height of the joystick's effective area
        let halfWidth = rect.width / 2 + offsetWidth;
        let halfHeight = rect.height / 2 + offsetHeight;

        //Calculate normalized delta values for X and Y movement
        let dx = (clientX - rect.left + offsetWidth) / halfWidth - 1;
        let dy = (clientY - rect.top + offsetHeight) / halfHeight - 1;

        //Calculate the squared radius of the movement vector
        let radiusSquared = dx * dx + dy * dy;

        //If the radius squared is greater than 1, normalize the vector
        if (radiusSquared > 1) {
            let radius = Math.sqrt(radiusSquared);
            dx /= radius;
            dy /= radius;
        }

        //Calculate new positions for the joystick head and line
        let newX = (dx + 1) * halfWidth - offsetWidth;
        let newY = (dy + 1) * halfHeight - offsetHeight;
        
        //Update the joystick head's position and the line's end point
        joystickHead.style.left = newX + 'px';
        joystickHead.style.top = newY + 'px';
        joystickLine.setAttribute('x2', newX + 'px');
        joystickLine.setAttribute('y2', newY + 'px');

        updateMovement(dx, dy);
    };
    
    //Used to move WALL-E
    document.addEventListener('mouseup', function(event) {
        if (!joystickDragged) return;

        movementStick.classList.remove('dragging');
        joystickDragged = false;
        joystickHead.style.left = '50%';
        joystickHead.style.top = '50%';

        joystickLine.setAttribute('x2', '50%');
        joystickLine.setAttribute('y2', '50%');
        updateMovement(0, 0);
    });

    //Used to move WALL-E
    document.addEventListener('touchend', function(event) {
        if (!joystickDragged) return;

        movementStick.classList.remove('dragging');
        joystickDragged = false;
        joystickHead.style.left = '50%';
        joystickHead.style.top = '50%';
        
        joystickLine.setAttribute('x2', '50%');
        joystickLine.setAttribute('y2', '50%');
        updateMovement(0, 0);
    });

    //Used to move WALL-E
    movementStick.onmousedown = function(event) {
        event.preventDefault();

        if(event.button === 2) return;
        if(movementStick.classList.contains('disabled')) return;

        movementStick.classList.add('dragging');
        joystickDragged = true;
        handleJoystick(event.clientX, event.clientY);
    };

    //Used to move WALL-E
    movementStick.ontouchstart = function(event) {
        event.preventDefault();
        
        if (movementStick.classList.contains('disabled')) return;

        movementStick.classList.add('dragging');
        joystickDragged = true;
        handleJoystick(event.targetTouches[0].clientX, event.targetTouches[0].clientY);
    };

    //Used to move WALL-E
    document.addEventListener('mousemove', function(event) {
        if (!joystickDragged) return;

        event.preventDefault();

        handleJoystick(event.clientX, event.clientY);
    });

    //Used to move WALL-E
    document.addEventListener('touchmove', function(event) {
        if (!joystickDragged) return;

        event.preventDefault();

        handleJoystick(event.targetTouches[0].clientX, event.targetTouches[0].clientY);
    });

    updateBatteryPercent();
});

var moveHeadRequestRunning = false;
var moveHeadPrevDx = 0;
var moveHeadPrevDy = 0;
var moveHeadNextDx = 0;
var moveHeadNextDy = 0;

const threshold = 5;

function moveHead(currentX, currentY) {
    moveHeadNextDx = currentX;
    moveHeadNextDy = currentY;

    if (moveHeadRequestRunning) return;
    if (moveHeadPrevDx === moveHeadNextDx && moveHeadPrevDy === moveHeadNextDy) return;

    moveHeadRequestRunning = true;

    moveHeadPrevDx = moveHeadNextDx;
    moveHeadPrevDy = moveHeadNextDy;

    const route = '/moveHead';

    var diffX = initialX - currentX;
    var diffY = initialY - currentY;

    var xDirection = 'none';
    var yDirection = 'none';

    if (Math.abs(diffX) > threshold) {
        xDirection = diffX > 0 ? 'right' : 'left';
    }

    if (Math.abs(diffY) > threshold) {
        yDirection = diffY > 0 ? 'down' : 'up';
    }

    fetch(route + '?xDirection=' + xDirection + '&yDirection=' + yDirection)
        .then(function(response) {
            if (response.ok) console.log('WALL-E moved his head.');
            else console.log('An error occurred while calling /moveHead.');
        })

        .then(function() {
            moveHeadRequestRunning = false;
            updateMovement(nextDx, nextDy);
        });
}

var moveRequestRunning = false;
var prevDx = 0;
var prevDy = 0;
var nextDx = 0;
var nextDy = 0;

//Used to move WALL-E
function updateMovement(dx, dy) {
    nextDx = dx;
    nextDy = dy;

    if (moveRequestRunning) return;
    if (prevDx === nextDx && prevDy === nextDy) return;

    moveRequestRunning = true;

    prevDx = nextDx;
    prevDy = nextDy;

    const leftSensorValue = document.getElementById('leftSensor');
    const rightSensorValue = document.getElementById('rightSensor');
    const route = '/move';

    Promise.all([
        fetch(route + '?xDirection=' + dx + '&yDirection=' + -dy)
            .then(function(response) {
                if (response.ok) console.log('WALL-E moved.');
                else console.log('An error occurred while calling /move.');
            }),

        fetch('/getSensorsValue')
            .then(function(response) {
                return response.json();
            })
            .then(function(data) {
                if(data[0] < 2 || data[0] > 400) leftSensorValue.textContent = 'Left sensor: no obstacle';
                else leftSensorValue.textContent = 'Left sensor: ' + data[0] + 'cm';

                if(data[1] < 2 || data[1] > 400) rightSensorValue.textContent = 'Right sensor: no obstacle';
                else rightSensorValue.textContent = 'Right sensor: ' + data[1] + 'cm';
            })
    ]).then(function() {
        moveRequestRunning = false;
        updateMovement(nextDx, nextDy);
    });
}

//Switch on/off light
function switchLight() {
    const lightButton = document.getElementById('lightButton');

    //Light state recovery
    fetch('/switchLight')
        .then(function(response) {
            return response.json();
        })
        .then(function(data) {
            if(data.light_on) {
                console.log('Light on.');
                lightButton.style.backgroundImage = 'url(\'static/images/bulbOn2.png\')';
            }
            
            else {
                console.log('Light off.');
                lightButton.style.backgroundImage = 'url(\'static/images/bulbOff2.png\')';
            }
        });
}

//Show/hide AI overlay
function switchAI(){
    const AIButton = document.getElementById('AIButton');
    const cameraDisplay = document.getElementById('cameraDisplay');

    if(AIOn == false) {
        AIOn = true;

        AIButton.style.backgroundImage = 'url(\'static/images/AIOn2.png\')';
        cameraDisplay.src = '/showCameraWithAIOverlay';

        console.log('Camera display with overlay.');
    } else {
        AIOn = false;

        AIButton.style.backgroundImage = 'url(\'static/images/AIOff2.png\')';
        cameraDisplay.src = '/showCameraWithoutAIOverlay';

        console.log('Camera display without overlay.');
    }
}

//Fonction activation/désactivation checkbox
function switchOperatingMode(){
    const checkbox = document.getElementById('autoCheckbox');
    const movementStick = document.getElementById('movementStick');
    const armSliders = Array.from(document.getElementsByClassName('armSlider'));
    const lightButton = document.getElementById('lightButton');

    fetch('/switchOperatingMode')
        .then(function(response) {
            return response.json();
        })
        .then(function(data) {
            if (data.auto_mode) {
                removeCameraDragActiveClass();
                lightButton.disabled = true;
                lightButton.style.backgroundImage = 'url(\'static/images/bulbOff2.png\')';
                movementStick.classList.add('disabled');

                armSliders.forEach(function(slider) {
                    slider.disabled = true;
                    slider.value = 0;
                });
            } else {
                setCameraDragActiveClass();
                lightButton.disabled = false;
                movementStick.classList.remove('disabled');

                armSliders.forEach(function(slider) {
                    slider.disabled = false;
                });
            }
        });
}

function move(direction, event) {
    event.preventDefault();

    const leftSensorValue = document.getElementById('leftSensor');
    const rightSensorValue = document.getElementById('rightSensor');
    const route = '/move';

    fetch(route + '?direction=' + direction)
        .then(function(response) {
            if (response.ok) {
                console.log('WALL-E moved.');
            } else {
                console.log('An error occurred while calling /move.');
            }
        });

    fetch('/getSensorsValue')
        .then(function(response) {
            return response.json();
        })
        .then(function(data) {
            if(data[0] < 2 || data[0] > 400) leftSensorValue.textContent = 'Left sensor: no obstacle';
            else leftSensorValue.textContent = 'Left sensor: ' + data[0] + 'cm';

            if(data[1] < 2 || data[1] > 400) rightSensorValue.textContent = 'Right sensor: no obstacle';
            else rightSensorValue.textContent = 'Right sensor: ' + data[1] + 'cm';
        });
}

function moveArm(arm, angle) {
    const route = '/moveArm';

    if(arm == 'left'){
        angle = 180 - angle;
    }

    fetch(route + '?arm=' + arm + '&angle=' + angle)
        .then(function(response) {
            if(response.ok) console.log('WALL-E moved an arm.');
            else console.log('An error occurred while calling /moveArm.');
        });
}

function makeSound(soundNum) {
    var sound = document.getElementById('sound' + soundNum);
    var progressBar = document.getElementById('progressBar' + soundNum);
    var soundDuration = sound.duration;
    var transitionDuration = soundDuration * 1000;

    progressBar.style.transition = 'width ' + transitionDuration + 'ms linear';
    progressBar.style.width = '100%';

    sound.volume = 0.05;
    sound.play();

    sound.addEventListener('ended', function() {
        progressBar.style.transition = 'width 0ms linear';
        progressBar.style.width = '0%';
    });

    const route = '/makeSound';

    fetch(route + '?sound=' + soundNum + '&duration=' + soundDuration)
        .then(function(response) {
            if(response.ok) console.log('WALL-E made the sound.');
            else console.log('An error occurred while calling /makeSound.');
        });
}

function mouseCameraDragActiveListener(event){
    initialX = event.clientX;
    initialY = event.clientY;
    cameraDragged = true;

    event.preventDefault();
}

function touchCameraDragActiveListener(event){
    initialX = event.touches[0].clientX;
    initialY = event.touches[0].clientY;
    cameraDragged = true;
    
    event.preventDefault();
}

function setCameraDragActiveClass(){
    cameraDisplay.classList.add('cameraDragActive');

    const cameraDragActive = document.getElementsByClassName('cameraDragActive')[0];

    cameraDragActive.addEventListener('mousedown', mouseCameraDragActiveListener);
    cameraDragActive.addEventListener('touchstart', touchCameraDragActiveListener);
    cameraDragActive.setAttribute('title', 'DRAG TO MOVE WALL-E\'S HEAD');
}

function removeCameraDragActiveClass(){
    const cameraDragActive = document.getElementsByClassName('cameraDragActive')[0];

    if(cameraDragActive != undefined){
        cameraDragActive.removeEventListener('mousedown', mouseCameraDragActiveListener);
        cameraDragActive.removeEventListener('touchstart', touchCameraDragActiveListener);
        cameraDragActive.removeAttribute('title');

        cameraDisplay.classList.remove('cameraDragActive');
    }
}

function updateBatteryPercent(){
    const batteryProgress = document.getElementById('batteryProgress');
    const batteryText = document.getElementById('batteryPercent');

    //Battery percent recovery/display
    fetch('/getBatteryPercent')
        .then(function(response) {
            return response.text();
        })
        .then(function(data) {            
            var batteryPercent = parseInt(data);
            var batteryHeight = batteryPercent * 18 / 100;

            batteryProgress.style.height = batteryHeight + 'px';
            batteryText.textContent = batteryPercent + '%';

            //Battery color change
            if(batteryPercent < 20) batteryProgress.style.background = 'red';
            else batteryProgress.style.background = 'green';
        });
}

//Battery percent display update (every minute)
setInterval(updateBatteryPercent, 60000);