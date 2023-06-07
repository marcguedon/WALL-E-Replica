var AIOn = false;

//Initialisation page web
document.addEventListener('DOMContentLoaded', function() {
    const checkbox = document.getElementById('autoCheckbox');
    const movementsButtons = Array.from(document.getElementsByClassName('movementButton'));
    const lightButton = document.getElementById('lightButton');

    //Récupération/affichage mode actif
    fetch('/getOperatingMode')
        .then(function(response) {
            return response.json();
        })
        .then(function(data) {
            //Si mode auto activé
            if (data.auto_mode) {
                checkbox.checked = true;
                lightButton.disabled = true;
                movementsButtons.forEach(function(button) {
                    button.disabled = true;
                });

                console.log('WALL-E est en mode auto.');
            }
            
            //Si mode manuel activé
            else {
                checkbox.checked = false;
                lightButton.disabled = false;
                movementsButtons.forEach(function(button) {
                    button.disabled = false;
                });

                console.log('WALL-E est en mode manuel.');
            }

            //Récupération état lumière
            fetch('/getLightState')
                .then(function(response) {
                    return response.json();
                })
                .then(function(data) {
                    //Si lumière allumée
                    if(data.light_on) {
                        lightButton.style.backgroundImage = 'url(\'static/images/bulbOn2.png\')';
                    }
                    
                    //Si lumière éteinte
                    else {
                        lightButton.style.backgroundImage = 'url(\'static/images/bulbOff2.png\')';
                    }
                });
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
            if(batteryPercent < 20) {
                batteryProgress.style.background = 'red';
            } else {
                batteryProgress.style.background = 'green';
            }

            batteryProgress.style.height = batteryHeight + 'px';
            batteryText.textContent = batteryPercent + '%';
        });     
});

//Fonction allumer/éteindre lumière
function switchLight() {
    const lightButton = document.getElementById('lightButton');

    //Récupération état lumière
    fetch('/getLightState')
        .then(function(response) {
            return response.json();
        })
        .then(function(data) {
            //Si lumière allumée
            if(data.light_on) {
                lightButton.style.backgroundImage = 'url(\'static/images/bulbOff2.png\')';

                fetch('/setLightOff')
                    .then(function(response) {
                        if (response.ok) {
                            console.log('WALL-E a éteint sa lumière.');
                        } else {
                            console.log('Une erreur s\'est produite lors de l\'appel à /setLightOff.');
                        }
                    });
            }
            
            //Si lumière éteinte
            else {
                lightButton.style.backgroundImage = 'url(\'static/images/bulbOn2.png\')';

                fetch('/setLightOn')
                    .then(function(response) {
                        if (response.ok) {
                            console.log('WALL-E a allumé sa lumière.');
                        } else {
                            console.log('Une erreur s\'est produite lors de l\'appel à /setLightOn.');
                        }
                    });
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
    const movementsButtons = Array.from(document.getElementsByClassName('movementButton'));
    const lightButton = document.getElementById('lightButton');

    //Mode auto
    if(checkbox.checked){
        lightButton.disabled = true;
        movementsButtons.forEach(function(button) {
            button.disabled = true;
        });

        fetch('/setAutoMode')
        .then(function(response) {
            if (response.ok) {
                console.log('WALL-E est en mode auto.');
            } else {
                console.log('Une erreur s\'est produite lors de l\'appel à /setAutoMode.');
            }
        });
    }

    //Mode manuel
    else{
        lightButton.disabled = false;
        movementsButtons.forEach(function(button) {
            button.disabled = false;
        });

        fetch('/setManualMode')
        .then(function(response) {
            if (response.ok) {
                console.log('WALL-E est en mode manuel.');
            } else {
                console.log('Une erreur s\'est produite lors de l\'appel à /setManualMode.');
            }
        });
    }
}

//Fonction Avancer
function moveForward() {
    fetch('/moveForward')
        .then(function(response) {
            if (response.ok) {
                console.log('WALL-E a avancé.');
            } else {
                console.log('Une erreur s\'est produite lors de l\'appel à /moveForward.');
            }
        });
}

//Fonction Tourner à gauche
function moveLeft() {
    fetch('/moveLeft')
        .then(function(response) {
            if (response.ok) {
                console.log('WALL-E a tourné à gauche.');
            } else {
                console.log('Une erreur s\'est produite lors de l\'appel à /moveLeft.');
            }
        });
}

//Fonction Tourner à droite
function moveRight() {
    fetch('/moveRight')
        .then(function(response) {
            if (response.ok) {
                console.log('WALL-E a tourné à droite.');
            } else {
                console.log('Une erreur s\'est produite lors de l\'appel à /moveRight.');
            }
        });
}

//Fonction Reculer
function moveBackward() {
    fetch('/moveBackward')
        .then(function(response) {
            if (response.ok) {
                console.log('WALL-E a reculé.');
            } else {
                console.log('Une erreur s\'est produite lors de l\'appel à /moveBackward.');
            }
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
    //sound.volume = 0;
    sound.play();

    sound.addEventListener('ended', function() {
        progressBar.style.transition = 'width 0ms linear';
        progressBar.style.width = '0%';
    });

    const route = '/makeSound';

    fetch(route + '?sound=' + soundNum)
        .then(function(response) {
            if (response.ok) {
                console.log('WALL-E a lancé le son.');
            } else {
                console.log('Une erreur s\'est produite lors de l\'appel à /makeSound.');
            }
        });
}