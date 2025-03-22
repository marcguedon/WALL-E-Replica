import pygame
import pygame._sdl2.audio as sdl2_audio


def get_devices(capture_devices: bool = False) -> tuple[str, ...]:
    init_by_me = not pygame.mixer.get_init()
    if init_by_me:
        pygame.mixer.init()
    devices = tuple(sdl2_audio.get_audio_device_names(capture_devices))
    if init_by_me:
        pygame.mixer.quit()
    return devices


print(get_devices())

# import pygame

# # Forcer l'utilisation d'ALSA
# pygame.mixer.init(
#     # frequency=44100, size=-16, channels=2, buffer=512, devicename="hw:2,0"
# )  # Remplacer par le bon périphérique si nécessaire
# print("pygame.mixer initialisé avec succès !")
# pygame.mixer.quit()
