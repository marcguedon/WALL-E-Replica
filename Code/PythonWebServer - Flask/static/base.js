//Initialisation page web
document.addEventListener('DOMContentLoaded', function() {
    const checkbox = document.getElementById('autoCheckbox');
    const buttons = Array.from(document.getElementsByClassName('movementButton'));

    fetch('/getOperatingMode')
        .then(function(response) {
            return response.json();
        })
        .then(function(data) {
            //Si mode auto activé
            if (data.auto_mode) {
                checkbox.checked = true;
                buttons.forEach(function(button) {
                    button.disabled = true;
                });
                console.log('WALL-E est en mode auto.');
            }
            
            //Si mode manuel activé
            else {
                checkbox.checked = false;
                buttons.forEach(function(button) {
                    button.disabled = false;
                });
                console.log('WALL-E est en mode manuel.');
            }
        });
});

//Fonction activation/désactivation checkbox
function switchOperatingMode(){
    const checkbox = document.getElementById('autoCheckbox');
    const buttons = Array.from(document.getElementsByClassName('movementButton'));

    //Mode auto
    if(checkbox.checked){
        buttons.forEach(function(button) {
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
        buttons.forEach(function(button) {
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

function makeSound(soundNum) {
    const route = "/makeSound";

    console.log(route + "?sound=" + soundNum);

    fetch(route + "?sound=" + soundNum)
        .then(function(response) {
            if (response.ok) {
                console.log('WALL-E a lancé le son.');
            } else {
                console.log('Une erreur s\'est produite lors de l\'appel à /makeSound.');
            }
        });
}