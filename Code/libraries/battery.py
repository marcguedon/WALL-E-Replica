import ADS1x15

ADS = ADS1x15.ADS1115(1, 0x48)

ADS.setGain(ADS.PGA_4_096V)

# 12.6V -> 20195
# 9V -> 14460

def getBatteryPercent():
    value = ADS.readADC(0)
    percent = (value - 14460) * (100 - 0) / (20195 - 14460)

    return percent