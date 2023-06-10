from pygame import mixer

mixer.init()

def playSound(soundNum):
    mixer.music.load('webServer/static/sounds/son' + str(soundNum) + '.mp3')
    mixer.music.set_volume(0.2)
    mixer.music.play()