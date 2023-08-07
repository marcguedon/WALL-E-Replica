var AIOn = false;
var initialX = null;
var initialY = null;
var isDragged = false;
var joystickDragged = false;

//Initialisation page web
document.addEventListener('DOMContentLoaded', function() {
    const checkbox = document.getElementById('autoCheckbox');
    const movementStick = document.getElementById('movementStick');
    const armSliders = Array.from(document.getElementsByClassName('armSlider'));
    const lightButton = document.getElementById('lightButton');

    //Récupération/affichage mode actif
    fetch('/getOperatingMode')
        .then(function(response) {
            return response.json();
        })
        .then(function(data) {
            //Si mode auto activé
            if (data.auto_mode) {
                removeCameraDragActiveClass();
                checkbox.checked = true;
                lightButton.disabled = true;
                movementStick.add('disabled');

                armSliders.forEach(function(slider) {
                    slider.disabled = true;
                    slider.value = 0;
                });

                console.log('WALL-E est en mode auto.');
            }
            
            //Si mode manuel activé
            else {
                setCameraDragActiveClass();
                checkbox.checked = false;
                lightButton.disabled = false;
                movementStick.classList.remove('disabled');

                armSliders.forEach(function(slider) {
                    slider.disabled = false;
                });

                console.log('WALL-E est en mode manuel.');
            }
        });

        //Récupération état lumière
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

    const batteryProgress = document.getElementById('batteryProgress');
    const batteryText = document.getElementById('batteryPercent');

    //Récupération/affichage pourcentage batterie
    fetch('/getBatteryPercent')
        .then(function(response) {
            return response.text();
        })
        .then(function(data) {            
            var batteryPercent = parseInt(data);
            var batteryHeight = batteryPercent * 18 / 100;

            //Changement couleur batterie
            if(batteryPercent < 20) batteryProgress.style.background = 'red';
            else batteryProgress.style.background = 'green';

            batteryProgress.style.height = batteryHeight + 'px';
            batteryText.textContent = batteryPercent + '%';
        });

    
    document.addEventListener('mousemove', function(event) {
        if (!isDragged) return;

        // Empêcher le défilement de la page lors du glissement
        event.preventDefault();

        moveHead(event.clientX, event.clientY);
    });

    document.addEventListener('touchmove', function(event) {
        if (!isDragged) return;

        // Empêcher le défilement de la page lors du glissement
        event.preventDefault();

        moveHead(event.touches[0].clientX, event.touches[0].clientY);
    });
    
    document.addEventListener('mouseup', function(event) {
        isDragged = false;
    });

    document.addEventListener('touchend', function(event) {
        isDragged = false;
    });

    joystickHead = movementStick.getElementsByClassName('joystickHead')[0];
    joystickLine = movementStick.getElementsByClassName('joystickLine')[0];

    let handleJoystick = function(clientX, clientY) {
        let rect = movementStick.getBoundingClientRect();
        let headRect = joystickHead.getBoundingClientRect();

        let ratio = 0.25;

        let offsetWidth = headRect.width * (ratio - 0.5);
        let offsetHeight = headRect.height * (ratio - 0.5);

        let halfWidth = rect.width / 2 + offsetWidth;
        let halfHeight = rect.height / 2 + offsetHeight;

        let dx = (clientX - rect.left + offsetWidth) / halfWidth - 1;
        let dy = (clientY - rect.top + offsetHeight) / halfHeight - 1;

        let radiusSquared = dx * dx + dy * dy;

        if (radiusSquared > 1) {
            let radius = Math.sqrt(radiusSquared);
            dx /= radius;
            dy /= radius;
        }

        let newX = (dx + 1) * halfWidth - offsetWidth;
        let newY = (dy + 1) * halfHeight - offsetHeight;
        
        joystickHead.style.left = newX + 'px';
        joystickHead.style.top = newY + 'px';

        joystickLine.setAttribute('x2', newX + 'px');
        joystickLine.setAttribute('y2', newY + 'px');

        updateMovement(dx, dy);
    };
    
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

    movementStick.onmousedown = function(event) {
        event.preventDefault();

        if(event.button === 2) return;
        if(movementStick.classList.contains('disabled')) return;

        movementStick.classList.add('dragging');
        joystickDragged = true;
        handleJoystick(event.clientX, event.clientY);
    };

    movementStick.ontouchstart = function(event) {
        event.preventDefault();
        
        if (movementStick.classList.contains('disabled')) return;

        movementStick.classList.add('dragging');
        joystickDragged = true;
        handleJoystick(event.targetTouches[0].clientX, event.targetTouches[0].clientY);
    };

    document.addEventListener('mousemove', function(event) {
        if (!joystickDragged) return;

        event.preventDefault();

        handleJoystick(event.clientX, event.clientY);
    });

    document.addEventListener('touchmove', function(event) {
        if (!joystickDragged) return;

        event.preventDefault();

        handleJoystick(event.targetTouches[0].clientX, event.targetTouches[0].clientY);
    });
});

var moveHeadRequestRunning = false;
var moveHeadPrevDx = 0;
var moveHeadPrevDy = 0;
var moveHeadNextDx = 0;
var moveHeadNextDy = 0;

function moveHead(currentX, currentY) {
    moveHeadNextDx = currentX;
    moveHeadNextDy = currentY;

    if (moveHeadRequestRunning) return;
    if (moveHeadPrevDx === moveHeadNextDx && moveHeadPrevDy === moveHeadNextDy) return;

    moveHeadRequestRunning = true;

    moveHeadPrevDx = moveHeadNextDx;
    moveHeadPrevDy = moveHeadNextDy;

    const route = '/head';

    var diffX = initialX - currentX;
    var diffY = initialY - currentY;

    var xDirection = 'none';
    var yDirection = 'none';

    // Détection de l'axe horizontal (swipe à gauche ou à droite)
    if (diffX > 0) xDirection = 'right';
    else if (diffX < 0) xDirection = 'left';

    // Détection de l'axe vertical (swipe vers le haut ou le bas)
    if (diffY > 0) yDirection = 'down';
    else if (diffY < 0) yDirection = 'up';

    fetch(route + '?xDirection=' + xDirection + '&yDirection=' + yDirection)
        .then(function(response) {
            if (response.ok) console.log('WALL-E a tourné sa tête.');
            else console.log('Une erreur s\'est produite lors de l\'appel à /head dans la direction ' + direction + '.');
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

    console.log('move: ' + dx + ', ' + dy);

    Promise.all([
        fetch(route + '?xDirection=' + dx + '&yDirection=' + -dy)
            .then(function(response) {
                if (response.ok) console.log('WALL-E a bougé.');
                else console.log('Une erreur s\'est produite lors de l\'appel à /move dans la direction ' + direction + '.');
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

//Fonction allumer/éteindre lumière
function switchLight() {
    const lightButton = document.getElementById('lightButton');

    //Récupération état lumière
    fetch('/switchLight')
        .then(function(response) {
            return response.json();
        })
        .then(function(data) {
            //Si lumière allumée
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

//Fonction afficher/cacher overlay IA
function switchAI(){
    const AIButton = document.getElementById('AIButton');
    const cameraDisplay = document.getElementById('cameraDisplay');

    if(AIOn == false) {
        AIOn = true;

        AIButton.style.backgroundImage = 'url(\'static/images/AIOn2.png\')';
        cameraDisplay.src = '/showCameraWithAIOverlay';

        console.log('Affichage caméra avec overlay.');
    } else {
        AIOn = false;

        AIButton.style.backgroundImage = 'url(\'static/images/AIOff2.png\')';
        cameraDisplay.src = '/showCameraWithoutAIOverlay';

        console.log('Affichage caméra sans overlay.');
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

//Fonction Bouger
function move(direction, event) {
    event.preventDefault();

    const leftSensorValue = document.getElementById('leftSensor');
    const rightSensorValue = document.getElementById('rightSensor');
    const route = '/move';

    fetch(route + '?direction=' + direction)
        .then(function(response) {
            if (response.ok) {
                console.log('WALL-E a bougé.');
            } else {
                console.log('Une erreur s\'est produite lors de l\'appel à /move dans la direction ' + direction + '.');
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

//Fonction Bouger bras
function moveArm(arm, angle) {
    const route = '/moveArm';

    if(arm == 'left'){
        angle = 180 - angle;
    }

    fetch(route + '?arm=' + arm + '&angle=' + angle)
        .then(function(response) {
            if(response.ok) console.log('WALL-E a bougé un bras.');
            else console.log('Une erreur s\'est produite lors de l\'appel à /moveArm à l\'angle ' + angle + '.');
        });
}

//Fonction Lancer son
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

    fetch(route + '?sound=' + soundNum)
        .then(function(response) {
            if(response.ok) console.log('WALL-E a lancé le son.');
            else console.log('Une erreur s\'est produite lors de l\'appel à /makeSound.');
        });
}

function mouseCameraDragActiveListener(event){
    initialX = event.clientX;
    initialY = event.clientY;
    isDragged = true;

    event.preventDefault();
}

function touchCameraDragActiveListener(event){
    initialX = event.touches[0].clientX;
    initialY = event.touches[0].clientY;
    isDragged = true;
    
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