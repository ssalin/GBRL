def GetSM50Concentration(reading,devno):
    # function to convert from analog read to ozone concentration
    # values returned are in parts per billion (PPB)
    # and should be in the range of 0-.15
    # math that's commented out is part of the derivation

    if (devno ==1): # need to measure resistances for more accuracy
        R1 = 10E3   # these are just the nominal values
        R2 = 18E3 
    else: #implied dev_2 since we only have 2 SM50s            
        R1 = 10E3 # should be a little different between devices
        R2 = 18E3

    # from voltage divider we know that
    # VO = VI * R2/(R1+R2)
    VO = 3.3 * reading/1023 # voltage on pin. ADC outputs 0-1023 for 0-3.3v
    # VO * (R1 + R2) = VI * R2
    VI = R1*VO/R2 + VO # voltage coming out of SM50

    concentration = (.015*VI/5)

    #concentration = (.15 * ((R1*VO/R2)+VO))/5
    return concentration

