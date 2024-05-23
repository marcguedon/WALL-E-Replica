from pygame import mixer

mixer.init()

# Play sound
def playSound(soundPath):
    mixer.music.load(soundPath)
    mixer.music.set_volume(0.2)
    mixer.music.play()