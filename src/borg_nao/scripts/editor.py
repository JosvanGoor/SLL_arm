
class Editor:
  
  f = None ##file
  canRead = None
  rhu = 255
  rhl = 0
  rsu = 255
  rsl = 0
  rvu = 255
  rvl = 0
  yhu = 255
  yhl = 0
  ysu = 255
  ysl = 0
  yvu = 255
  yvl = 0
  bhu = 255
  bhl = 0
  bsu = 255
  bsl = 0
  bvu = 255
  bvl = 0
  
  
  def __init__(self, filename):

    try:
      self.f = open(filename, 'r+')
    except:
      self.f = open(filename, 'w')
      self.writeInit()
      self.writeRed(255, 0, 255, 0, 255, 0)
      self.writeBlue(255, 0, 255, 0, 255, 0)
      self.writeYellow(255, 0, 255, 0, 255, 0)
      self.writeEnd()
      self.f.close()
      self.f = open(filename, 'r+')
    self.readParams()
    

  def writeInit(self):
    initXML = "\
<launch>\n\
\t<node name=\"colorblob\" pkg=\"borg_nao\" type=\"colorblob_node\" output=\"screen\" > \n\
\t\t<param name=\"image_topic\" value=\"/NAO/image_bottom\" />\n\
\t\t<param name=\"visual_mode\" value=\"true\"/>\n\n"
    self.f.write(initXML)

  def writeEnd(self):
    endingXML = "\
\t\t<param name=\"def_hue_upper\" value=\"0\" />\n\
\t\t<param name=\"def_hue_lower\" value=\"0\" />\n\
\t\t<param name=\"def_saturation_upper\" value=\"0\" />\n\
\t\t<param name=\"def_saturation_lower\" value=\"0\" />\n\
\t\t<param name=\"def_value_upper\" value=\"0\" />\n\
\t\t<param name=\"def_value_lower\" value=\"0\"/>\n\
\t</node>\n\
</launch>\n"
    self.f.write(endingXML)

  def writeBlue(self, hu, hl, su, sl, vu, vl):
    blue = "\
\t\t<param name=\"blue_hue_upper\" value=\"" + str(hu) + "\" />\n\
\t\t<param name=\"blue_hue_lower\" value=\"" + str(hl) + "\" />\n\
\t\t<param name=\"blue_saturation_upper\" value=\"" + str(su) + "\" />\n\
\t\t<param name=\"blue_saturation_lower\" value=\"" + str(sl) + "\" />\n\
\t\t<param name=\"blue_value_upper\" value=\"" + str(vu) + "\" />\n\
\t\t<param name=\"blue_value_lower\" value=\"" + str(vl) + "\"/>\n\n\
"
    self.f.write(blue)
    
    
  def writeYellow(self, hu, hl, su, sl, vu, vl):
    yellow = "\
\t\t<param name=\"yellow_hue_upper\" value=\"" + str(hu) + "\" />\n\
\t\t<param name=\"yellow_hue_lower\" value=\"" + str(hl) + "\" />\n\
\t\t<param name=\"yellow_saturation_upper\" value=\"" + str(su) + "\" />\n\
\t\t<param name=\"yellow_saturation_lower\" value=\"" + str(sl) + "\" />\n\
\t\t<param name=\"yellow_value_upper\" value=\"" + str(vu) + "\" />\n\
\t\t<param name=\"yellow_value_lower\" value=\"" + str(vl) + "\"/>\n\n\
"
    self.f.write(yellow)
    

  def writeRed(self, hu, hl, su, sl, vu, vl):
    red = "\
\t\t<param name=\"red_hue_upper\" value=\"" + str(hu) + "\" />\n\
\t\t<param name=\"red_hue_lower\" value=\"" + str(hl) + "\" />\n\
\t\t<param name=\"red_saturation_upper\" value=\"" + str(su) + "\" />\n\
\t\t<param name=\"red_saturation_lower\" value=\"" + str(sl) + "\" />\n\
\t\t<param name=\"red_value_upper\" value=\"" + str(vu) + "\" />\n\
\t\t<param name=\"red_value_lower\" value=\"" + str(vl) + "\"/>\n\n\
"
    self.f.write(red)

  def getValue(self,str, cue):
    idx = str.find(cue, 0, len(str))
    val = None
    try:
      val = int(str[idx + len(cue):idx + len(cue) + 3])
    except:
      try:
        val = int(str[idx + len(cue):idx + len(cue) + 2])
      except:
        try:
          val = int(str[idx + len(cue):idx + len(cue) + 1])
        except:
          print "Error parsing value, while searching for ", cue
          val = -1
    return val
    
  def readRed(self):
    self.f.seek(0,0)
    huCue = "<param name=\"red_hue_upper\" value=\""  
    hlCue = "<param name=\"red_hue_lower\" value=\""
    suCue = "<param name=\"red_saturation_upper\" value=\""
    slCue = "<param name=\"red_saturation_lower\" value=\""
    vuCue = "<param name=\"red_value_upper\" value=\""
    vlCue = "<param name=\"red_value_lower\" value=\""
    
    str = self.f.read()
    hu = self.getValue(str, huCue)
    hl = self.getValue(str, hlCue)
    su = self.getValue(str, suCue)
    sl = self.getValue(str, slCue)
    vu = self.getValue(str, vuCue)
    vl = self.getValue(str, vlCue)
    return hu, hl, su, sl, vu, vl
    
  def readBlue(self):
    self.f.seek(0,0)
    huCue = "<param name=\"blue_hue_upper\" value=\""  
    hlCue = "<param name=\"blue_hue_lower\" value=\""
    suCue = "<param name=\"blue_saturation_upper\" value=\""
    slCue = "<param name=\"blue_saturation_lower\" value=\""
    vuCue = "<param name=\"blue_value_upper\" value=\""
    vlCue = "<param name=\"blue_value_lower\" value=\""
      
    str = self.f.read()
    hu = self.getValue(str, huCue)
    hl = self.getValue(str, hlCue)
    su = self.getValue(str, suCue)
    sl = self.getValue(str, slCue)
    vu = self.getValue(str, vuCue)
    vl = self.getValue(str, vlCue)
    return hu, hl, su, sl, vu, vl

  def readYellow(self):
    self.f.seek(0,0)
    huCue = "<param name=\"yellow_hue_upper\" value=\""  
    hlCue = "<param name=\"yellow_hue_lower\" value=\""
    suCue = "<param name=\"yellow_saturation_upper\" value=\""
    slCue = "<param name=\"yellow_saturation_lower\" value=\""
    vuCue = "<param name=\"yellow_value_upper\" value=\""
    vlCue = "<param name=\"yellow_value_lower\" value=\""
    
    str = self.f.read()
    hu = self.getValue(str, huCue)
    hl = self.getValue(str, hlCue)
    su = self.getValue(str, suCue)
    sl = self.getValue(str, slCue)
    vu = self.getValue(str, vuCue)
    vl = self.getValue(str, vlCue)
    return hu, hl, su, sl, vu, vl
  
  def readParams(self):
    self.rhu, self.rhl, self.rsu, self.rsl, self.rvu, self.rvl = self.readRed()
    self.yhu, self.yhl, self.ysu, self.ysl, self.yvu, self.yvl = self.readYellow()
    self.bhu, self.bhl, self.bsu, self.bsl, self.bvu, self.bvl = self.readBlue()
    
  def writeParams(self):
    
    self.f.seek(0,0)
    self.writeInit()
    self.writeRed(self.rhu, self.rhl, self.rsu, self.rsl, self.rvu, self.rvl)
    self.writeYellow(self.yhu, self.yhl, self.ysu, self.ysl, self.yvu, self.yvl)
    self.writeBlue(self.bhu, self.bhl, self.bsu, self.bsl, self.bvu, self.bvl)
    self.writeEnd()
    
  def getRed(self):
    return self.rhu, self.rhl, self.rsu, self.rsl, self.rvu, self.rvl
  
  def getYellow(self):
    return self.yhu, self.yhl, self.ysu, self.ysl, self.yvu, self.yvl
  
  def getBlue(self):
    return self.bhu, self.bhl, self.bsu, self.bsl, self.bvu, self.bvl
  
  def setRed(self, hu, hl, su, sl, vu, vl):
    self.rhu = hu
    self.rhl = hl 
    self.rsu = su
    self.rsl = sl 
    self.rvu = vu
    self.rvl = vl

  def setYellow(self, hu, hl, su, sl, vu, vl):
    self.yhu = hu
    self.yhl = hl 
    self.ysu = su
    self.ysl = sl 
    self.yvu = vu
    self.yvl = vl
    
  def setBlue(self, hu, hl, su, sl, vu, vl):
    self.bhu = hu
    self.bhl = hl 
    self.bsu = su
    self.bsl = sl 
    self.bvu = vu
    self.bvl = vl
    
  def closeFile(self):
    self.f.close()
    
