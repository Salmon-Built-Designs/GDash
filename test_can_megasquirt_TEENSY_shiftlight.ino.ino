  
/*   pixels.setPixelColor(0, pixels.Color(255,0,0)) ; //vert
   pixels.setPixelColor(1, pixels.Color(0,255,0)) ; //rouge
   pixels.setPixelColor(2, pixels.Color(0,0,255)) ;  //bleu     
   pixels.setPixelColor(3, pixels.Color(255,255,0)) ; //jaune
   pixels.setPixelColor(4, pixels.Color(0,255,255)) ; //violet
   pixels.setPixelColor(5, pixels.Color(255,255,255)) ; //blanc
   pixels.setPixelColor(4, pixels.Color(0,255,127)) ; //rose
   pixels.setPixelColor(5, pixels.Color(127,255,0)) ; //orange
*/

#include <NextionPage.h>
#include <SoftwareSerial.h>
#include <NextionNumber.h>
#include <NextionVariableNumeric.h>
#include <NextionText.h>
#include <Adafruit_NeoPixel.h>

SoftwareSerial HMI(9, 10); // RX, TX
Nextion nex(HMI);
 
#include <FlexCAN.h>
#include <kinetis_flexcan.h>

FlexCAN CANbus(500000);
static CAN_message_t rxmsg;

int Page,PageFIX,BtnCarto,BtnSequ,readvalue;
int var3,var6,var7;
int va20,va21,va22,va23,va24,va25,va26;
int vaN_LED1get,vaN_LED2get,vaN_LED3get,vaN_LED4get,vaN_LED5get,vaN_LED6get,vaN_LED7get,vaN_LED8get,vaN_LED9get,vaN_LED10get,vaN_LED11get;
int vaCOLOR_LED1get,vaCOLOR_LED2get,vaCOLOR_LED3get,vaCOLOR_LED4get,vaCOLOR_LED5get,vaCOLOR_LED6get,vaCOLOR_LED7get,vaCOLOR_LED8get,vaCOLOR_LED9get,vaCOLOR_LED10get,vaCOLOR_LED11get;
int vaLumiLedget;
int i = 0;
int va10get,va11get,va12get,va13get,va14get,va15get,va16get,va17get;
int pageid;
byte indicator[8],PA0,PA0var,PM2,PM2var,PE0,PE4,PE1,Flag,Flag1,Flag2,Flag3,Flag4,PE1var; // where to store indicator data
float val,tension;
double var1,var0,var2,var4,var5,BATTV,IAC,dwell,idle_tar,AFRtgt,AFR,BATTVvar,old,ADC6;
double oldRPM=0,alpha=.9;
double oldtension=0,beta=.9;
unsigned int MAP,SPKADV,RPM,RPMvar,injduty,Baro,PW1,nexAFR,nexCLT,TPS,TPSvar,MAT,MATvar,CLT,CLTvar;

 // int pinreset=15; 
String Etat0 = {"OFF"}, Etat1 = {"ON"}, Etat2 = {"less"}, Etat3 = {"more"}, Etat4 = {"ok"}, EtatMoteur, p10, EtatVentiloPA0, EtatVentiloPM2, EtatBtnSequentielPE0, EtatFuelPumpPE4, EtatBtnCartoPE1, EtatBtnDepartADC6;
String EtatCLT;

#define PIN            12 // Which pin on the Arduino is connected to the NeoPixels?
#define NUMPIXELS      10  // How many NeoPixels are attached to the Arduino? 
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800); // pour WS2812
/*
NextionNumber numbern0(nex, 0, 1, "va6");
NextionNumber va10(nex, 0, 5, "va10");
NextionNumber va11(nex, 0, 6, "va11");
NextionNumber va12(nex, 0, 9, "va12");
NextionNumber va13(nex, 0, 10, "va13");
NextionNumber va14(nex, 0, 11, "va14");
NextionNumber va15(nex, 0, 12, "va15");
NextionNumber va16(nex, 0, 13, "va16");
NextionNumber va17(nex, 0, 14, "va17");
*/
NextionNumber numbern0(nex, 0, 1, "va6");
NextionNumber va10(nex, 0, 3, "va10");
NextionNumber va11(nex, 0, 4, "va11");
NextionNumber va12(nex, 0, 5, "va12");
NextionNumber va13(nex, 0, 6, "va13");
NextionNumber va14(nex, 0, 7, "va14");
NextionNumber va15(nex, 0, 8, "va15");
NextionNumber va16(nex, 0, 9, "va16");
NextionNumber va17(nex, 0, 10, "va17");

NextionNumber vaN_LED1(nex, 0, 11, "vaN_LED1");
NextionNumber vaN_LED2(nex, 0, 12, "vaN_LED2");
NextionNumber vaN_LED3(nex, 0, 13, "vaN_LED3");
NextionNumber vaN_LED4(nex, 0, 14, "vaN_LED4");
NextionNumber vaN_LED5(nex, 0, 15, "vaN_LED5");
NextionNumber vaN_LED6(nex, 0, 16, "vaN_LED6");
NextionNumber vaN_LED7(nex, 0, 17, "vaN_LED7");
NextionNumber vaN_LED8(nex, 0, 18, "vaN_LED8");
NextionNumber vaN_LED9(nex, 0, 19, "vaN_LED9");
NextionNumber vaN_LED10(nex, 0, 20, "vaN_LED10");
NextionNumber vaN_LED11(nex, 0, 21, "vaN_LED11");

NextionNumber vaCOLOR_LED1(nex, 0, 22, "vaCOLOR_LED1");
NextionNumber vaCOLOR_LED2(nex, 0, 23, "vaCOLOR_LED2");
NextionNumber vaCOLOR_LED3(nex, 0, 24, "vaCOLOR_LED3");
NextionNumber vaCOLOR_LED4(nex, 0, 25, "vaCOLOR_LED4");
NextionNumber vaCOLOR_LED5(nex, 0, 26, "vaCOLOR_LED5");
NextionNumber vaCOLOR_LED6(nex, 0, 27, "vaCOLOR_LED6");
NextionNumber vaCOLOR_LED7(nex, 0, 28, "vaCOLOR_LED7");
NextionNumber vaCOLOR_LED8(nex, 0, 29, "vaCOLOR_LED8");
NextionNumber vaCOLOR_LED9(nex, 0, 30, "vaCOLOR_LED9");
NextionNumber vaCOLOR_LED10(nex, 0, 31, "vaCOLOR_LED10");
NextionNumber vaCOLOR_LED11(nex, 0, 32, "vaCOLOR_LED11");

NextionNumber vaLumiLed(nex, 0, 34, "vaLumiLed");

NextionPage page0      (nex, 0,  0, "page0");
NextionPage pageconfig        (nex, 1,  0, "pageconfig");
NextionPage page7000 (nex, 2,  0, "page7000");
NextionPage page8000     (nex, 3,  0, "page8000");
NextionPage page9000denis        (nex, 4,  0, "page9000denis");
NextionPage pageconfigled     (nex, 5,  0, "pageconfigled");
NextionPage pageconfigBV       (nex, 6,  0, "pageconfigBV");
NextionPage pagemenu    (nex, 7,  0, "pagemenu");
NextionPage pageconfigDysp      (nex, 8,  0, "pageconfigDysp");
NextionPage pageminimaxi       (nex, 9,  0, "pageminimaxi");


//unsigned int Color(byte r,byte g,byte b);
uint32_t couleur1,couleur2,couleur3,couleur4,couleur5,couleur6,couleur7,couleur8,couleur9,couleur10,couleur11;
uint32_t RPMLED1,RPMLED2,RPMLED3,RPMLED4,RPMLED5,RPMLED6,RPMLED7,RPMLED8,RPMLED9,ALERTE_Poil,ALERTE_Teau;
uint32_t vert = pixels.Color(255,0,0);
uint32_t rouge = pixels.Color(0,255,0);
uint32_t jaune = pixels.Color(255,255,0);
uint32_t orange = pixels.Color(127,255,0);
uint32_t bleu = pixels.Color(0,0,255);
uint32_t violet = pixels.Color(0,255,255);

//  uint32_t val;
//*******************************************************************************************************************
void setup()
{
   pixels.begin(); // This initializes the NeoPixel library.
   effacer_led ();
   //delay(1000);
  Serial.begin(115200);
  HMI.begin(115200);
  nex.init();
  CANbus.begin();
  CLTvar=0;
  TPSvar=0;
  MATvar=0;
  BATTVvar=0;  
Flag4=0;


   // config for the Neopixel 
   pinMode(12, OUTPUT);
//pinMode(pinreset, OUTPUT); 
//pinMode(13, OUTPUT); 

}

//elapsedMillis DisplayTime; //Establish a timer to prevent unnecessary screen rewrites
//*******************************************************************************************************************
void loop() {
  while(HMI.available())
   HMI.read();

pageid = nex.getCurrentPage();
Serial.print("  pageid: ");Serial.println(pageid);Serial.println();

    if (Flag==0) {
      Serial.print("  Flag: ");Serial.print(Flag);Serial.println();
      while(Page==0){Page=numbern0.getValue();}
      va10get=va10.getValue();  
      va11get=va11.getValue(); 
      va12get=va12.getValue(); 
      va13get=va13.getValue(); 
      va14get=va14.getValue(); 
      va15get=va15.getValue(); 
      va16get=va16.getValue(); 
      va17get=va17.getValue(); 
      vaCOLOR_LED1get=vaCOLOR_LED1.getValue();  
      vaCOLOR_LED2get=vaCOLOR_LED2.getValue(); 
      vaCOLOR_LED3get=vaCOLOR_LED3.getValue();
      vaCOLOR_LED4get=vaCOLOR_LED4.getValue(); 
      vaCOLOR_LED5get=vaCOLOR_LED5.getValue(); 
      vaCOLOR_LED6get=vaCOLOR_LED6.getValue(); 
      vaCOLOR_LED7get=vaCOLOR_LED7.getValue(); 
      vaCOLOR_LED8get=vaCOLOR_LED8.getValue(); 
      vaCOLOR_LED9get=vaCOLOR_LED9.getValue(); 
      vaCOLOR_LED10get=vaCOLOR_LED10.getValue(); 
      vaCOLOR_LED11get=vaCOLOR_LED11.getValue();
      vaN_LED1get=vaN_LED1.getValue();
      vaN_LED2get=vaN_LED2.getValue();
      vaN_LED3get=vaN_LED3.getValue();
      vaN_LED4get=vaN_LED4.getValue();
      vaN_LED5get=vaN_LED5.getValue();
      vaN_LED6get=vaN_LED6.getValue();
      vaN_LED7get=vaN_LED7.getValue();
      vaN_LED8get=vaN_LED8.getValue();
      vaN_LED9get=vaN_LED9.getValue();
      vaN_LED10get=vaN_LED10.getValue();
      vaN_LED11get=vaN_LED11.getValue();
      vaLumiLedget=vaLumiLed.getValue();
      Serial.print("  va10: ");Serial.print(va10get);Serial.println();
      Serial.print("  va11: ");Serial.print(va11get);Serial.println();
      Serial.print("  va12: ");Serial.print(va12get);Serial.println();
      Serial.print("  va13: ");Serial.print(va13get);Serial.println();
      Serial.print("  va14: ");Serial.print(va14get);Serial.println();
      Serial.print("  va15: ");Serial.print(va15get);Serial.println();
      Serial.print("  va16: ");Serial.print(va16get);Serial.println();
      Serial.print("  va17: ");Serial.print(va17get);Serial.println();
      
      Serial.print("  vaN_LED1get: ");Serial.print(vaN_LED1get);Serial.println();
      Serial.print("  vaN_LED2get: ");Serial.print(vaN_LED2get);Serial.println();
      Serial.print("  vaN_LED3get: ");Serial.print(vaN_LED3get);Serial.println();
      Serial.print("  vaN_LED4get: ");Serial.print(vaN_LED4get);Serial.println();
      Serial.print("  vaN_LED5get: ");Serial.print(vaN_LED5get);Serial.println();
      Serial.print("  vaN_LED6get: ");Serial.print(vaN_LED6get);Serial.println();
      Serial.print("  vaN_LED7get: ");Serial.print(vaN_LED7get);Serial.println();
      Serial.print("  vaN_LED8get: ");Serial.print(vaN_LED8get);Serial.println();
      Serial.print("  vaN_LED9get: ");Serial.print(vaN_LED9get);Serial.println();
      Serial.print("  vaN_LED10get: ");Serial.print(vaN_LED10get);Serial.println();
      Serial.print("  vaN_LED11get: ");Serial.print(vaN_LED11get);Serial.println();
      Serial.print("  vaCOLOR_LED1get: ");Serial.print(vaCOLOR_LED1get);Serial.println();
      Serial.print("  vaCOLOR_LED2get: ");Serial.print(vaCOLOR_LED2get);Serial.println();
      Serial.print("  vaCOLOR_LED3get: ");Serial.print(vaCOLOR_LED3get);Serial.println();
      Serial.print("  vaCOLOR_LED4get: ");Serial.print(vaCOLOR_LED4get);Serial.println();
      Serial.print("  vaCOLOR_LED5get: ");Serial.print(vaCOLOR_LED5get);Serial.println();
      Serial.print("  vaCOLOR_LED6get: ");Serial.print(vaCOLOR_LED6get);Serial.println();
      Serial.print("  vaCOLOR_LED7get: ");Serial.print(vaCOLOR_LED7get);Serial.println();
      Serial.print("  vaCOLOR_LED8get: ");Serial.print(vaCOLOR_LED8get);Serial.println();
      Serial.print("  vaCOLOR_LED9get: ");Serial.print(vaCOLOR_LED9get);Serial.println();
      Serial.print("  vaCOLOR_LED10get: ");Serial.print(vaCOLOR_LED10get);Serial.println();
      Serial.print("  vaCOLOR_LED11get: ");Serial.print(vaCOLOR_LED11get);Serial.println();
      Serial.print("  page: ");Serial.print(Page);Serial.println();
      Serial.print("  Flag: ");Serial.print(Flag);Serial.println();
      Flag=1;  
      }
    colorLed();
    shift();
    Serial.println();  
    val = analogRead(0);
    Serial.print("  valbrut: ");Serial.print(val,0);Serial.println();
    tension=val*(5.0/1023.0);
    tension=tension*1000;
    tension = (int) tension;
        //tension = 0.1*oldtension + (1-0.1)*tension;
    //oldtension = tension;  
    //tension=(tension+5)/10;
    //tension=tension*10;
    Serial.print("  tension: ");Serial.print(tension);Serial.println();
    HMI.print("pageconfigBV.n100.val=");
    HMI.print(tension,0);
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff);
    Rapport();
    CurrentPage();
    Serial.print("  PageFIX: ");Serial.print(PageFIX);Serial.println();
    if ( CANbus.read(rxmsg) ){
        Serial.print("  CANbus.read: ok ");Serial.println();
        LectureCan();
        if (Flag4==0){
            for (int i=0; i <= 30; i++){
            LectureCan();
            delay(100);
            }
            Affichage0();
            }
            
        if (Flag1==0){Affichage0();}
        Flag1=1;
                
        Serial.println(RPM,DEC);
        Serial.print("  PageFIX: ");Serial.print(PageFIX);Serial.println();
        switch (PageFIX) { // 
          case 2: 
          var4=7000;
          var6=309;
          var1=51; //echelle maxi rpm2
          var7=41; // precision rpm2
          Affichage2();
          break;
        case 3: 
          var4=8000;
          var6=310;
          var1=58;//57
          var7=41;//40      
          Affichage3();
          break;
        case 4: 
          var4=9000;
          var6=310;
          var1=65;//65
          var7=42;//40
     
          Affichage4();
          break;
        }  
      }
      else { Flag1=0;Eteint(); }  

}

//*******************************************************************************************************************
void Rapport()
{
  
  va20=va11get-va10get;
  va20=va20/2;
  va20=va20+va10get;

  va21=va12get-va11get;
  va21=va21/2;
  va21=va21+va11get;

  va22=va13get-va12get;
  va22=va22/2;
  va22=va22+va12get;

  va23=va14get-va13get;
  va23=va23/2;
  va23=va23+va13get;

  va24=va15get-va14get;
  va24=va24/2;
  va24=va24+va14get;

  va25=va16get-va15get;
  va25=va25/2;
  va25=va25+va15get;

  va26=va17get-va16get;
  va26=va26/2;
  va26=va26+va16get;
        Serial.print("  va20 : ");Serial.print(va20);Serial.println(); 
  // R    *********************************

  if(tension>=0 && tension<va20)//  
    {
    HMI.print("rapport.txt=");
    HMI.write("\"");
    HMI.print("R");
    HMI.write("\"");
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff);
      delay(100);
    }
  // N    *********************************
   if(tension>=va20 && tension<va21)
    {
    nex.sendCommand("rapport.xcen=0");
    HMI.print("rapport.txt=");
    HMI.write("\"");
    HMI.print("N");
    HMI.write("\"");
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff);
            delay(100);
    }
  // 1    *********************************
  if(tension>=va21 && tension<va22)
    {
    nex.sendCommand("rapport.xcen=2");
    HMI.print("rapport.txt=");
    HMI.write("\"");
    HMI.print("1");
    HMI.write("\"");
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff);
    }
  // 2    *********************************
  if(tension>=va22 && tension<va23)
    {
    nex.sendCommand("rapport.xcen=0");
    
    HMI.print("rapport.txt=");
    HMI.write("\"");
    HMI.print("2");
    HMI.write("\"");
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff);
    }
  // 3    *********************************    
  if(tension>=va23 && tension<va24)
    {
    HMI.print("rapport.txt=");
    HMI.write("\"");
    HMI.print("3");
    HMI.write("\"");
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff);
    }
  // 4    *********************************    
  if(tension>=va24 && tension<va25)
    {
    HMI.print("rapport.txt=");
    HMI.write("\"");
    HMI.print("4");
    HMI.write("\"");
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff);
    }
  // 5    *********************************
  if(va17get<=va16get+50 && va17get>= va16get-50)
    {
    Flag2=1;
    } 
  else Flag2=0;
     Serial.print("  flag2 : ");Serial.print(Flag2);Serial.println();
     Serial.print("  tension : ");Serial.print(tension);Serial.println();     
  if(tension>=va25 && tension<va26 or tension>=va25 && tension<5000 && Flag2==1)
    {
    HMI.print("rapport.txt=");
    HMI.write("\"");
    HMI.print("5");
    HMI.write("\"");
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff);
    }
    
  // 6    *********************************
  if(tension>=va26 && tension<5000 && Flag2==0)
    {
    HMI.print("rapport.txt=");
    HMI.write("\"");
    HMI.print("6");
    HMI.write("\"");
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff);
    }
  if(tension<0 or tension>5000)
    {
    HMI.print("rapport.txt=");
    HMI.write("\"");
    HMI.print("E ");
    HMI.write("\"");
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff);
    }
    
}
//*******************************************************************************************************************
void Eteint()
{
    if (Flag1==0){
    HMI.print("t9.txt=");
    HMI.write("\"");
    HMI.print("--");
    HMI.write("\"");
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff); 
      
    HMI.print("t5.txt=");
    HMI.write("\"");
    HMI.print("--");
    HMI.write("\"");
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff); 

    HMI.print("t7.txt=");
    HMI.write("\"");
    HMI.print("--");
    HMI.write("\"");
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff); 

    HMI.print("t8.txt=");
    HMI.write("\"");
    HMI.print("--");
    HMI.write("\"");
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff); 
      
    RPM =0;
    HMI.print("RPM_bar1.val=");
    HMI.print(0);
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff);
    HMI.print("RPM_bar2.val=");
    HMI.print(0);
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff);
            
    HMI.print("rpm_val.txt=");
    HMI.write("\"");
    HMI.print("----");
    HMI.write("\"");
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff);  
    }
}

//*******************************************************************************************************************
void Affichage0()
{
    Serial.print("  flag4: ");Serial.print(Flag4);Serial.println();


    HMI.print("t9.txt=");//page9000denis.
    HMI.write("\"");
    HMI.print(CLT);
    HMI.write("\"");
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff);   


    HMI.print("t7.txt=");
    HMI.write("\"");
    HMI.print(MAT);
    HMI.write("\"");
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff); 
      
    HMI.print("t5.txt=");
    HMI.write("\"");
    HMI.print(TPS);
    HMI.write("\"");
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff); 

    //tension=4930/1023;
  // BATT 

    HMI.print("t8.txt=");
    HMI.write("\"");
    HMI.print(BATTV,1);
    HMI.write("\"");
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff);
  
  // RPM  
  
    RPM = alpha*oldRPM + (1-alpha)*RPM;
    oldRPM = RPM;
    RPM=(RPM+5)/5;
    RPM=RPM*5;
    if (RPM<=50){RPM=0;}
    
    HMI.print("rpm_val.txt=");
    HMI.write("\"");
    HMI.print(RPM);
    HMI.write("\"");
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff);






/*
    
    HMI.print("page9000denis.t9.txt=");
    HMI.write("\"");
    HMI.print(CLT);
    HMI.write("\"");
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff); 
      
    HMI.print("page9000denis.t5.txt=");
    HMI.write("\"");
    HMI.print(TPS);
    HMI.write("\"");
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff); 
      
      if (MAT!=MATvar){
    HMI.print("page9000denis.t7.txt=");
    HMI.write("\"");
    HMI.print(MAT);
    HMI.write("\"");
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff); 
       MATvar=MAT; 
       }
/*
    HMI.print("page9000denis.t7.txt=");
    HMI.write("\"");
    HMI.print(MAT);
    HMI.write("\"");
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff); 

    HMI.print("page9000denis.t8.txt=");
    HMI.write("\"");
    HMI.print(BATTV);
    HMI.write("\"");
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff); 
 */     
      HMI.print("RPM_bar1.val=");
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff);
    HMI.print("RPM_bar2.val=");
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff);
 /*           
    HMI.print("page9000denis.rpm_val.txt=");
    HMI.write("\"");
    HMI.print(RPM);
    HMI.write("\"");
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff);  
 */     
      Flag4=1;
       Serial.print("  Tair: ");Serial.print(MAT);Serial.println();
              Serial.print("  Tairbar: ");Serial.print(MATvar);Serial.println();
       Serial.print("  Teau: ");Serial.print(CLT);Serial.println();
                     Serial.print("  Teaubar: ");Serial.print(CLTvar);Serial.println();
       Serial.print("  TPS: ");Serial.print(TPS);Serial.println(); 
                     Serial.print("  Tpsbar: ");Serial.print(TPSvar);Serial.println();
       Serial.print("  BATT: ");Serial.print(BATTV, 1);Serial.println();

}
//*******************************************************************************************************************
void LectureCan()
{
     switch (rxmsg.id) { // ID's 1520+ are Megasquirt CAN broadcast frames. EAch frame represents a data group http://www.msextra.com/doc/pdf/Megasquirt_CAN_Broadcast.pdf
      case 1520: // Group 0
        RPM = (int)(word(rxmsg.buf[6], rxmsg.buf[7]));
        //PW1 = (float)(word(rxmsg.buf[2], rxmsg.buf[3]));
        //injduty = ((PW1 / 1000 * RPM / 120) / 10);
        break;
      case 1521: // Group 1
        SPKADV = (float)(word(rxmsg.buf[0], rxmsg.buf[1]));
        indicator[0] = rxmsg.buf[3]; // engine 
        //AFRtgt = (float)(word(0x00, rxmsg.buf[4]));
        break;
      case 1522: // Group 2
        //Baro = (float)(word(rxmsg.buf[0], rxmsg.buf[1]));
        //MAP = (float)(word(rxmsg.buf[2], rxmsg.buf[3]));
        MAT = (int)(word(rxmsg.buf[4], rxmsg.buf[5]));
        MAT = ((MAT-320)*0.555555555)/10;
        CLT = (int)(word(rxmsg.buf[6], rxmsg.buf[7]));
        CLT = ((CLT-320)*0.555555555)/10;
        //nexCLT = (float)(word(rxmsg.buf[6], rxmsg.buf[7]));
        break;
      case 1523: // Group 3
        TPS = (int)(word(rxmsg.buf[0], rxmsg.buf[1]));
        TPS = TPS/10;
        BATTV = (float)(word(rxmsg.buf[2], rxmsg.buf[3]));
        BATTV=BATTV/10;
        //AFR = (float)(word(rxmsg.buf[4], rxmsg.buf[5]));
        //nexAFR = (float)(word(rxmsg.buf[4], rxmsg.buf[5]));
        break;
      case 1524: // Group 4
        break;
      case 1526: // Group 6
        IAC = (float)(word(rxmsg.buf[6], rxmsg.buf[7])); //IAC = (IAC * 49) / 125;
      case 1529: // 9
        //dwell = (float)(word(rxmsg.buf[4], rxmsg.buf[5]));
        break;
      case 1530: // Group 10
        indicator[1] = rxmsg.buf[0]; // status 1
        indicator[2] = rxmsg.buf[1]; // status 2
        indicator[3] = rxmsg.buf[2]; // status 3
        indicator[6] = rxmsg.buf[6]; // status 6
        indicator[7] = rxmsg.buf[7]; // status 7
        break;
      case 1535: // Group 15 adc6;adc7
        ADC6 = (float)(word(rxmsg.buf[0], rxmsg.buf[1]));
        break;
      case 1548: // Group 28
        //idle_tar = (float)(word(rxmsg.buf[0], rxmsg.buf[1]));
        break;
      case 1571: // Group 51
        indicator[8] = rxmsg.buf[0]; // PORT_A   fan1 
        PA0 = bitRead(indicator[8],0);
        indicator[8] = rxmsg.buf[4]; //PORT_M2 IDLE  fan2
        PM2 = bitRead(indicator[8],2); 
        indicator[8] = rxmsg.buf[2]; //PORT_E0 FLEX   btn seq
        PE0 = bitRead(indicator[8],0); 
        indicator[8] = rxmsg.buf[2]; //PORT_E4 Fuel pulp
        PE4 = bitRead(indicator[8],4); 
        indicator[8] = rxmsg.buf[2]; //PORT_E0 FLEX   btn seq
        PE1 = bitRead(indicator[8],1); 
        break;
        }
     //Serial.print("  LectureCan: ok ");Serial.println();   
}     
//******************************************************************************************************************* 
void CurrentPage()
{
      if (Page==0) {
      PageFIX=0;
      }
      if (Page==1) {
      PageFIX=1;
      }
      if (Page==2) {
      PageFIX=2;
      }
      if (Page==3) {
      PageFIX=3;
      }
      if (Page==4) {
      PageFIX=4;
      }
}

//*******************************************************************************************************************
void Affichage() //70000
{
    var5=var4/1000; //var4=7000
    var5=var5*var6;//2170 var6=310
    var0=(RPM/var5);
    var0=var0*100;
    var2=RPM/var1; //var1=50  
    var0=round(var0);
       Serial.print("  PageFIX: ");Serial.print(PageFIX);Serial.println();
  // TEAU  CLTvar
          Serial.print("  Tair: ");Serial.print(MAT);Serial.println();
              Serial.print("  Tairbar: ");Serial.print(MATvar);Serial.println();
       Serial.print("  Teau: ");Serial.print(CLT);Serial.println();
                     Serial.print("  Teaubar: ");Serial.print(CLTvar);Serial.println();
       Serial.print("  TPS: ");Serial.print(TPS);Serial.println(); 
                     Serial.print("  Tpsbar: ");Serial.print(TPSvar);Serial.println();
     if (CLT!=CLTvar){
    HMI.print("t9.txt=");//page9000denis.
    HMI.write("\"");
    HMI.print(CLT);
    HMI.write("\"");
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff);  
       CLTvar=CLT; 
      } 

      if (MAT!=MATvar){
    HMI.print("t7.txt=");
    HMI.write("\"");
    HMI.print(MAT);
    HMI.write("\"");
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff); 
       MATvar=MAT; 
      } 
      
     if (TPS!=TPSvar){
    HMI.print("t5.txt=");
    HMI.write("\"");
    HMI.print(TPS);
    HMI.write("\"");
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff); 
       TPSvar=TPS; 
      } 

    //tension=4930/1023;
  // BATT 
  if (BATTVvar!=BATTV){   
    HMI.print("t8.txt=");
    HMI.write("\"");
    HMI.print(BATTV,1);
    HMI.write("\"");
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff);
      BATTVvar=BATTV;
  }
  
  // RPM  
  
    RPM = alpha*oldRPM + (1-alpha)*RPM;
    oldRPM = RPM;
    RPM=(RPM+5)/5;
    RPM=RPM*5;
    if (RPM<=50){RPM=0;}
 /*   
  if (RPMvar!=RPM){ 
    HMI.print("rpm_val.txt=");
    HMI.write("\"");
    HMI.print(RPM);
    HMI.write("\"");
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff);

  //  ConfigSuivantRPMmax();

  // RPM Barre Graph 1
  
    if (var0<=100)
    {
    HMI.print("RPM_bar1.val=");
    HMI.print(var0,0);
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff);
    HMI.print("RPM_bar2.val=");
    HMI.print(0);
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff);
    }
    
  // RPM Barre Graph 2
    if (var0>100)
    {
    HMI.print("RPM_bar1.val=");
    HMI.print(100);
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff);
    var2=var2-var7;
          // Serial.print("  var2-barre: ");Serial.print(var2,0);Serial.println();
    HMI.print("RPM_bar2.val=");
    HMI.print(var2,0);
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff);
    }
     RPMvar=RPM;
   }
*/   
  // Etat des indicateurs    

  //Etatindicateur();
//Serial.print("  pe1: ");Serial.print(PE1);Serial.println();
if (PE1var!=PE1){
       if (PE1==1)
        {
     HMI.print("t12.txt=");     
    HMI.write("\"");
    HMI.print("CARTO 1");
    HMI.write("\"");
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff);
    PE1var=PE1;
        } 
        else  
        {
     HMI.print("t12.txt=");     
    HMI.write("\"");
    HMI.print("CARTO 2");
    HMI.write("\"");
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff);
    PE1var=PE1;
        } 
        }
Serial.print("  pa0: ");Serial.print(PA0);Serial.println();
Serial.print("  pm2: ");Serial.print(PM2);Serial.println();
if (PA0var!=PA0 or PM2var!=PM2){
      if (PM2==0 && PA0==1)
        {
     HMI.print("t13.pco=");     
    //HMI.write("\"");
    HMI.print("2016");
    //HMI.write("\"");
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff);
    //PA0var=PA0;
    //PM2var=PM2;
        } 
        
         if (PM2==1 && PA0==1)
        {
     HMI.print("t13.pco=");     
    //HMI.write("\"");
    HMI.print("63488");
    //HMI.write("\"");
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff);
    //PA0var=PA0;
    //PM2var=PM2
        } 
              
        else if (PA0==0) 
        {
     HMI.print("t13.pco=");     
    //HMI.write("\"");
    HMI.print("50712");
    //HMI.write("\"");
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff);
    //PA0var=PA0;
        } 
    PA0var=PA0;
    PM2var=PM2;
        }
        
        //else EtatBtnCartoPE1=Etat1; 
    /*
    HMI.print("page9000.t11.txt=");
    HMI.write("\"");
    HMI.print(EtatMoteur);
    HMI.write("\"");
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff);
      */

    //Serial.print("  affichage4: ok ");Serial.println(); 
}      

//*******************************************************************************************************************



void Affichage2() //70000
{
    var5=var4/1000; //var4=7000
    var5=var5*var6;//2170 var6=310
    var0=(RPM/var5);
    var0=var0*100;
    var2=RPM/var1; //var1=50  
    var0=round(var0);
       Serial.print("  PageFIX: ");Serial.print(PageFIX);Serial.println();
  // TEAU  CLTvar
          Serial.print("  Tair: ");Serial.print(MAT);Serial.println();
              Serial.print("  Tairbar: ");Serial.print(MATvar);Serial.println();
       Serial.print("  Teau: ");Serial.print(CLT);Serial.println();
                     Serial.print("  Teaubar: ");Serial.print(CLTvar);Serial.println();
       Serial.print("  TPS: ");Serial.print(TPS);Serial.println(); 
                     Serial.print("  Tpsbar: ");Serial.print(TPSvar);Serial.println();
     if (CLT!=CLTvar){
    HMI.print("t9.txt=");//page9000denis.
    HMI.write("\"");
    HMI.print(CLT);
    HMI.write("\"");
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff);  
       CLTvar=CLT; 
      } 

      if (MAT!=MATvar){
    HMI.print("t7.txt=");
    HMI.write("\"");
    HMI.print(MAT);
    HMI.write("\"");
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff); 
       MATvar=MAT; 
      } 
      
     if (TPS!=TPSvar){
    HMI.print("t5.txt=");
    HMI.write("\"");
    HMI.print(TPS);
    HMI.write("\"");
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff); 
       TPSvar=TPS; 
      } 

    //tension=4930/1023;
  // BATT 
  if (BATTVvar!=BATTV){   
    HMI.print("t8.txt=");
    HMI.write("\"");
    HMI.print(BATTV,1);
    HMI.write("\"");
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff);
      BATTVvar=BATTV;
  }
  
  // RPM  
  
    RPM = alpha*oldRPM + (1-alpha)*RPM;
    oldRPM = RPM;
    RPM=(RPM+5)/5;
    RPM=RPM*5;
    if (RPM<=50){RPM=0;}
    
  if (RPMvar!=RPM){ 
    HMI.print("rpm_val.txt=");
    HMI.write("\"");
    HMI.print(RPM);
    HMI.write("\"");
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff);

  //  ConfigSuivantRPMmax();

  // RPM Barre Graph 1
  
    if (var0<=100)
    {
    HMI.print("RPM_bar1.val=");
    HMI.print(var0,0);
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff);
    HMI.print("RPM_bar2.val=");
    HMI.print(0);
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff);
    }
    
  // RPM Barre Graph 2
    if (var0>100)
    {
    HMI.print("RPM_bar1.val=");
    HMI.print(100);
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff);
    var2=var2-var7;
          // Serial.print("  var2-barre: ");Serial.print(var2,0);Serial.println();
    HMI.print("RPM_bar2.val=");
    HMI.print(var2,0);
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff);
    }
     RPMvar=RPM;
   }
  // Etat des indicateurs    

  //Etatindicateur();
//Serial.print("  pe1: ");Serial.print(PE1);Serial.println();
if (PE1var!=PE1){
       if (PE1==1)
        {
     HMI.print("t12.txt=");     
    HMI.write("\"");
    HMI.print("CARTO 1");
    HMI.write("\"");
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff);
    PE1var=PE1;
        } 
        else  
        {
     HMI.print("t12.txt=");     
    HMI.write("\"");
    HMI.print("CARTO 2");
    HMI.write("\"");
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff);
    PE1var=PE1;
        } 
        }
Serial.print("  pa0: ");Serial.print(PA0);Serial.println();
Serial.print("  pm2: ");Serial.print(PM2);Serial.println();
if (PA0var!=PA0 or PM2var!=PM2){
      if (PM2==0 && PA0==1)
        {
     HMI.print("t13.pco=");     
    //HMI.write("\"");
    HMI.print("2016");
    //HMI.write("\"");
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff);
    //PA0var=PA0;
    //PM2var=PM2;
        } 
        
         if (PM2==1 && PA0==1)
        {
     HMI.print("t13.pco=");     
    //HMI.write("\"");
    HMI.print("63488");
    //HMI.write("\"");
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff);
    //PA0var=PA0;
    //PM2var=PM2
        } 
              
        else if (PA0==0) 
        {
     HMI.print("t13.pco=");     
    //HMI.write("\"");
    HMI.print("50712");
    //HMI.write("\"");
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff);
    //PA0var=PA0;
        } 
    PA0var=PA0;
    PM2var=PM2;
        }
        
        //else EtatBtnCartoPE1=Etat1; 
    /*
    HMI.print("page9000.t11.txt=");
    HMI.write("\"");
    HMI.print(EtatMoteur);
    HMI.write("\"");
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff);
      */

    //Serial.print("  affichage4: ok ");Serial.println(); 
}      
//*******************************************************************************************************************
void Affichage3()
{
    var5=var4/1000; //var4=7000
    var5=var5*var6;//2170 var6=310
    var0=(RPM/var5);
    var0=var0*100;
    var2=RPM/var1; //var1=50  
    var0=round(var0);
       Serial.print("  PageFIX: ");Serial.print(PageFIX);Serial.println();
  // TEAU  CLTvar
          Serial.print("  Tair: ");Serial.print(MAT);Serial.println();
              Serial.print("  Tairbar: ");Serial.print(MATvar);Serial.println();
       Serial.print("  Teau: ");Serial.print(CLT);Serial.println();
                     Serial.print("  Teaubar: ");Serial.print(CLTvar);Serial.println();
       Serial.print("  TPS: ");Serial.print(TPS);Serial.println(); 
                     Serial.print("  Tpsbar: ");Serial.print(TPSvar);Serial.println();
     if (CLT!=CLTvar){
    HMI.print("t9.txt=");//page9000denis.
    HMI.write("\"");
    HMI.print(CLT);
    HMI.write("\"");
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff);  
       CLTvar=CLT; 
      } 

      if (MAT!=MATvar){
    HMI.print("t7.txt=");
    HMI.write("\"");
    HMI.print(MAT);
    HMI.write("\"");
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff); 
       MATvar=MAT; 
      } 
      
     if (TPS!=TPSvar){
    HMI.print("t5.txt=");
    HMI.write("\"");
    HMI.print(TPS);
    HMI.write("\"");
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff); 
       TPSvar=TPS; 
      } 

    //tension=4930/1023;
  // BATT 
  if (BATTVvar!=BATTV){   
    HMI.print("t8.txt=");
    HMI.write("\"");
    HMI.print(BATTV,1);
    HMI.write("\"");
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff);
      BATTVvar=BATTV;
  }
  
  // RPM  
  
    RPM = alpha*oldRPM + (1-alpha)*RPM;
    oldRPM = RPM;
    RPM=(RPM+5)/5;
    RPM=RPM*5;
    if (RPM<=50){RPM=0;}
    
  if (RPMvar!=RPM){ 
    HMI.print("rpm_val.txt=");
    HMI.write("\"");
    HMI.print(RPM);
    HMI.write("\"");
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff);

  //  ConfigSuivantRPMmax();

  // RPM Barre Graph 1
  
    if (var0<=100)
    {
    HMI.print("RPM_bar1.val=");
    HMI.print(var0,0);
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff);
    HMI.print("RPM_bar2.val=");
    HMI.print(0);
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff);
    }
    
  // RPM Barre Graph 2
    if (var0>100)
    {
    HMI.print("RPM_bar1.val=");
    HMI.print(100);
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff);
    var2=var2-var7;
          // Serial.print("  var2-barre: ");Serial.print(var2,0);Serial.println();
    HMI.print("RPM_bar2.val=");
    HMI.print(var2,0);
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff);
    }
     RPMvar=RPM;
   }
  // Etat des indicateurs    

  //Etatindicateur();
//Serial.print("  pe1: ");Serial.print(PE1);Serial.println();
if (PE1var!=PE1){
       if (PE1==1)
        {
     HMI.print("t12.txt=");     
    HMI.write("\"");
    HMI.print("CARTO 1");
    HMI.write("\"");
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff);
    PE1var=PE1;
        } 
        else  
        {
     HMI.print("t12.txt=");     
    HMI.write("\"");
    HMI.print("CARTO 2");
    HMI.write("\"");
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff);
    PE1var=PE1;
        } 
        }
Serial.print("  pa0: ");Serial.print(PA0);Serial.println();
Serial.print("  pm2: ");Serial.print(PM2);Serial.println();
if (PA0var!=PA0 or PM2var!=PM2){
      if (PM2==0 && PA0==1)
        {
     HMI.print("t13.pco=");     
    //HMI.write("\"");
    HMI.print("2016");
    //HMI.write("\"");
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff);
    //PA0var=PA0;
    //PM2var=PM2;
        } 
        
         if (PM2==1 && PA0==1)
        {
     HMI.print("t13.pco=");     
    //HMI.write("\"");
    HMI.print("63488");
    //HMI.write("\"");
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff);
    //PA0var=PA0;
    //PM2var=PM2
        } 
              
        else if (PA0==0) 
        {
     HMI.print("t13.pco=");     
    //HMI.write("\"");
    HMI.print("50712");
    //HMI.write("\"");
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff);
    //PA0var=PA0;
        } 
    PA0var=PA0;
    PM2var=PM2;
        }
        
        //else EtatBtnCartoPE1=Etat1; 
    /*
    HMI.print("page9000.t11.txt=");
    HMI.write("\"");
    HMI.print(EtatMoteur);
    HMI.write("\"");
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff);
      */

    //Serial.print("  affichage4: ok ");Serial.println(); 
}      
//*******************************************************************************************************************
void Affichage4()
{
    var5=var4/1000; //var4=7000
    var5=var5*var6;//2170 var6=310
    var0=(RPM/var5);
    var0=var0*100;
    var2=RPM/var1; //var1=50  
    var0=round(var0);
       Serial.print("  PageFIX: ");Serial.print(PageFIX);Serial.println();
  // TEAU  CLTvar
          Serial.print("  Tair: ");Serial.print(MAT);Serial.println();
              Serial.print("  Tairbar: ");Serial.print(MATvar);Serial.println();
       Serial.print("  Teau: ");Serial.print(CLT);Serial.println();
                     Serial.print("  Teaubar: ");Serial.print(CLTvar);Serial.println();
       Serial.print("  TPS: ");Serial.print(TPS);Serial.println(); 
                     Serial.print("  Tpsbar: ");Serial.print(TPSvar);Serial.println();
     if (CLT!=CLTvar){
    HMI.print("t9.txt=");//page9000denis.
    HMI.write("\"");
    HMI.print(CLT);
    HMI.write("\"");
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff);  
       CLTvar=CLT; 
      } 

      if (MAT!=MATvar){
    HMI.print("t7.txt=");
    HMI.write("\"");
    HMI.print(MAT);
    HMI.write("\"");
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff); 
       MATvar=MAT; 
      } 
      
     if (TPS!=TPSvar){
    HMI.print("t5.txt=");
    HMI.write("\"");
    HMI.print(TPS);
    HMI.write("\"");
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff); 
       TPSvar=TPS; 
      } 

    //tension=4930/1023;
  // BATT 
  if (BATTVvar!=BATTV){   
    HMI.print("t8.txt=");
    HMI.write("\"");
    HMI.print(BATTV,1);
    HMI.write("\"");
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff);
      BATTVvar=BATTV;
  }
  
  // RPM  
  
    RPM = alpha*oldRPM + (1-alpha)*RPM;
    oldRPM = RPM;
    RPM=(RPM+5)/5;
    RPM=RPM*5;
    if (RPM<=50){RPM=0;}
    
  if (RPMvar!=RPM){ 
    HMI.print("rpm_val.txt=");
    HMI.write("\"");
    HMI.print(RPM);
    HMI.write("\"");
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff);

  //  ConfigSuivantRPMmax();

  // RPM Barre Graph 1
  
    if (var0<=100)
    {
    HMI.print("RPM_bar1.val=");
    HMI.print(var0,0);
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff);
    HMI.print("RPM_bar2.val=");
    HMI.print(0);
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff);
    }
    
  // RPM Barre Graph 2
    if (var0>100)
    {
    HMI.print("RPM_bar1.val=");
    HMI.print(100);
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff);
    var2=var2-var7;
          // Serial.print("  var2-barre: ");Serial.print(var2,0);Serial.println();
    HMI.print("RPM_bar2.val=");
    HMI.print(var2,0);
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff);
    }
     RPMvar=RPM;
   }
  // Etat des indicateurs    

  //Etatindicateur();
//Serial.print("  pe1: ");Serial.print(PE1);Serial.println();
if (PE1var!=PE1){
       if (PE1==1)
        {
     HMI.print("t12.txt=");     
    HMI.write("\"");
    HMI.print("CARTO 1");
    HMI.write("\"");
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff);
    PE1var=PE1;
        } 
        else  
        {
     HMI.print("t12.txt=");     
    HMI.write("\"");
    HMI.print("CARTO 2");
    HMI.write("\"");
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff);
    PE1var=PE1;
        } 
        }
Serial.print("  pa0: ");Serial.print(PA0);Serial.println();
Serial.print("  pm2: ");Serial.print(PM2);Serial.println();
if (PA0var!=PA0 or PM2var!=PM2){
      if (PM2==0 && PA0==1)
        {
     HMI.print("t13.pco=");     
    //HMI.write("\"");
    HMI.print("2016");
    //HMI.write("\"");
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff);
    //PA0var=PA0;
    //PM2var=PM2;
        } 
        
         if (PM2==1 && PA0==1)
        {
     HMI.print("t13.pco=");     
    //HMI.write("\"");
    HMI.print("63488");
    //HMI.write("\"");
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff);
    //PA0var=PA0;
    //PM2var=PM2
        } 
              
        else if (PA0==0) 
        {
     HMI.print("t13.pco=");     
    //HMI.write("\"");
    HMI.print("50712");
    //HMI.write("\"");
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff);
    //PA0var=PA0;
        } 
    PA0var=PA0;
    PM2var=PM2;
        }
        
        //else EtatBtnCartoPE1=Etat1; 
    /*
    HMI.print("page9000.t11.txt=");
    HMI.write("\"");
    HMI.print(EtatMoteur);
    HMI.write("\"");
      HMI.write(0xff);
      HMI.write(0xff);
      HMI.write(0xff);
      */

    //Serial.print("  affichage4: ok ");Serial.println(); 
}   

//*******************************************************************************************************************
/*void ConfigSuivantRPMmax()
{
   //Variale suivant RPMmax
  switch (PageFIX) { // 
      case 2: 
      var4=7000;
      var6=309;
      var1=51; //echelle maxi rpm2
      var7=41; // precision rpm2
      break;

      case 3: 
      var4=8000;
      var6=310;
      var1=58;//57
      var7=41;//40
      break;

      case 4: 
      var4=9000;
      var6=310;
      var1=65;//65
      var7=42;//40
      break;
      }  
      
  // Calcul variable
    var5=var4/1000; //var4=7000
    var5=var5*var6;//2170 var6=310
    var0=(RPM/var5);
    var0=var0*100;
    var2=RPM/var1; //var1=50  
    var0=round(var0);
       Serial.print("  PageFIX: ");Serial.print(PageFIX);Serial.println();
       //Serial.print("  var0: ");Serial.print(var0,0);Serial.println();
      // Serial.print("  var4: ");Serial.print(var4,0);Serial.println();
      // Serial.print("  var5: ");Serial.print(var5,0);Serial.println();
         //  Serial.print("  var2: ");Serial.print(var2,0);Serial.println();
}

*/

//*******************************************************************************************************************
void Etatindicateur()
{
  /*
    if (indicator[0]!=0)
        {
        EtatMoteur=Etat1;
        nex.sendCommand("vis p10,1");
        } 
        else
        {
        EtatMoteur=Etat0;
        nex.sendCommand("vis p10,0");
        }
   Serial.print("  PORT_A0: ");Serial.print(PA0,DEC);Serial.println(); 
   */    
    /*if (PA0==1)
        {
        HMI.print("t12.txt=");
        HMI.write("\"");
        HMI.print("ON");
        HMI.write("\"");
          HMI.write(0xff);
          HMI.write(0xff);
          HMI.write(0xff);   
        //EtatVentiloPA0=Etat1;
        nex.sendCommand("vis p12,1");
        //delay (30);
        } */
   if (PA0!=1)
        {
        HMI.print("page7000.t12.txt=");
        HMI.write("\"");
        HMI.print("OFF");
        HMI.write("\"");
          HMI.write(0xff);
          HMI.write(0xff);
          HMI.write(0xff);   
        //EtatVentiloPA0=Etat0;
        nex.sendCommand("vis page7000.p12,0");
        //delay (30);
        }
        else {
        HMI.print("page.t12.txt=");
        HMI.write("\"");
        HMI.print("ON");
        HMI.write("\"");
          HMI.write(0xff);
          HMI.write(0xff);
          HMI.write(0xff);   
        //EtatVentiloPA0=Etat1;
        nex.sendCommand("vis page.p12,1");          
        }
    /*   
    if (BATTV>=12.5 & BATTV<=14)
        {
        nex.sendCommand("vis p1,0");  //rouge
        nex.sendCommand("vis p0,1"); // vert
        } 
    if (BATTV<12.5 || BATTV>14 )
        {
        nex.sendCommand("vis p0,0");//vert
        nex.sendCommand("vis p1,1"); //rouge
        }

        
    if (CLT<=75) { // 
        nex.sendCommand("vis p3,1");
        nex.sendCommand("vis p4,0");
        nex.sendCommand("vis p5,0");
      }
    if (CLT>75 & CLT<95) { // 
        nex.sendCommand("vis p3,0");
        nex.sendCommand("vis p4,1");
        nex.sendCommand("vis p5,0");
      }
    if (CLT>=95) { // 
        nex.sendCommand("vis p3,0");
        nex.sendCommand("vis p4,0");
        nex.sendCommand("vis p5,1");
      }
   */       
         
    if (PM2==1)
        {
        EtatVentiloPM2=Etat1; 
        } 
        else EtatVentiloPM2=Etat0; 
  /*        
    if (PE0==1)
        {
        EtatBtnSequentielPE0=Etat0; 
        } 
        else EtatBtnSequentielPE0=Etat1; 
    if (PE4==1)
        {
        EtatFuelPumpPE4=Etat1; 
        } 
        else EtatFuelPumpPE4=Etat0;   
        */      
     if (PE1==1)
        {
        EtatBtnCartoPE1=Etat0; 
        } 
        else EtatBtnCartoPE1=Etat1; 
        /*
     if (ADC6==5)
        {
        EtatBtnDepartADC6=Etat1; 
        } 
        else EtatBtnDepartADC6=Etat0;
        
        //Serial.print("  Etatindicateur: ok ");Serial.println();   
        */
        
}


//*******************************************************************************************************************


//*******************************************************************************************************************


//*******************************************************************************************************************
//void DebugSerial()
//;
       //Serial.println("MCP2515 Library Receive Example...");
       //Serial.print("ID: ");
       //Serial.print(rxId, DEC);
       //Serial.print("  var0: ");Serial.print(var0);Serial.println();
       //Serial.print("  var4: ");Serial.print(var4, DEC);Serial.println();
       //Serial.print("  var5: ");Serial.print(var5, DEC);Serial.println();
       //Serial.print("  var2: ");Serial.print(var2, DEC);Serial.println();
       //Serial.print("  RPM: ");Serial.print(RPM, DEC);Serial.println();
       //Serial.print("  Tair: ");Serial.print(MAT, 1);Serial.println();
       //Serial.print("  Teau: ");Serial.print(CLT, 1);Serial.println();
       //Serial.print("  TPS: ");Serial.print(TPS, 1);Serial.println(); 
       //Serial.print("  BATT: ");Serial.print(BATTV, 1);Serial.println();
       //Serial.print("  indicator: ");Serial.print(indicator[0], DEC);Serial.println();
       //Serial.print("  EtatMoteur: ");Serial.print(EtatMoteur);Serial.println();         
       //Serial.print("  indicator1: ");Serial.print(indicator[1], DEC);Serial.println();
       //Serial.print("  indicator2: ");Serial.print(indicator[2], DEC);Serial.println();
       //Serial.print("  indicator3: ");Serial.print(indicator[3], DEC);Serial.println();
       //Serial.print("  indicator6: ");Serial.print(indicator[6], DEC);Serial.println();
       //Serial.print("  indicator7: ");Serial.print(indicator[7], DEC);Serial.println();       
       //Serial.print("  indicator4: ");Serial.print(indicator[4], DEC);Serial.println(); 
       //Serial.print("  PORT_A0: ");Serial.print(PA0,DEC);Serial.println();
       //Serial.print("  EtatVentiloPA0: ");Serial.print(EtatVentiloPA0);Serial.println();         
       //Serial.print("  PORT_M2: ");Serial.print(PM2,DEC);Serial.println();
       //Serial.print("  EtatVentiloPM2: ");Serial.print(EtatVentiloPM2);Serial.println(); 
       //Serial.print("  PORT_E0: ");Serial.print(PE0,DEC);Serial.println();
       //Serial.print("  EtatBtnSequentielPE0: ");Serial.print(EtatBtnSequentielPE0);Serial.println();
       //Serial.print("  PORT_E4: ");Serial.print(PE4,DEC);Serial.println();
       //Serial.print("  EtatFuelPumpPE4: ");Serial.print(EtatFuelPumpPE4);Serial.println();
       //Serial.print("  PORT_E1: ");Serial.print(PE1,DEC);Serial.println();
       //Serial.print("  EtatBtnCartoPE1: ");Serial.print(EtatBtnCartoPE1);Serial.println();
       //Serial.print("  ADC6: ");Serial.print(ADC6, 1);Serial.println();
       //Serial.print("  EtatBtnDépartADC6: ");Serial.print(EtatBtnDepartADC6);Serial.println();

// *********************************************************   effacer_led      ********************************************************************
void effacer_led (){
                      for (i=0; i<8;i++) {
                            pixels.setPixelColor(i, pixels.Color(0,0,0));} // Moderately bright rouge color.
                            pixels.show();delay(10);
}

void cligo_Nmax (){
       for (int j=0; j<8;j++) {               
            pixels.setPixelColor( j, pixels.Color(0, 0, 0));}  // even pixels 0
            pixels.show();
            delay (10); 
       for (int j=0; j<8;j++) {               
            pixels.setPixelColor( j, couleur9);}  // even pixels bleue
            pixels.show(); 
            delay (10);  
}

void colorLed(){
  switch (vaCOLOR_LED1get) { // 
      case 2016: 
      couleur1=vert;
      break;
      case 63488: 
      couleur1=rouge;
      break;
      case 64832: 
      couleur1=jaune;
      break;
      case 31: 
      couleur1=bleu;
      break;
      case 32799: 
      couleur1=violet;
      break;
      }
  switch (vaCOLOR_LED2get) { // 
      case 2016: 
      couleur2=vert;
      break;

      case 63488: 
      couleur2=rouge;
      break;

      case 64832: 
      couleur2=jaune;
      break;

      case 31: 
      couleur2=bleu;
      break;

      case 32799: 
      couleur2=violet;
      break;
      }
  switch (vaCOLOR_LED3get) { // 
      case 2016: 
      couleur3=vert;
      break;

      case 63488: 
      couleur3=rouge;
      break;

      case 64832: 
      couleur3=jaune;
      break;

      case 31: 
      couleur3=bleu;
      break;

      case 32799: 
      couleur3=violet;
      break;
      }
  switch (vaCOLOR_LED4get) { // 
      case 2016: 
      couleur4=vert;
      break;

      case 63488: 
      couleur4=rouge;
      break;

      case 64832: 
      couleur4=jaune;
      break;

      case 31: 
      couleur4=bleu;
      break;

      case 32799: 
      couleur4=violet;
      break;
      }
  switch (vaCOLOR_LED5get) { // 
      case 2016: 
      couleur5=vert;
      break;

      case 63488: 
      couleur5=rouge;
      break;

      case 64832: 
      couleur5=jaune;
      break;

      case 31: 
      couleur5=bleu;
      break;

      case 32799: 
      couleur5=violet;
      break;
      }
  switch (vaCOLOR_LED6get) { // 
      case 2016: 
      couleur6=vert;
      break;

      case 63488: 
      couleur6=rouge;
      break;

      case 64832: 
      couleur6=jaune;
      break;

      case 31: 
      couleur6=bleu;
      break;

      case 32799: 
      couleur6=violet;
      break;
      }
  switch (vaCOLOR_LED7get) { // 
      case 2016: 
      couleur7=vert;
      break;

      case 63488: 
      couleur7=rouge;
      break;

      case 64832: 
      couleur7=jaune;
      break;

      case 31: 
      couleur7=bleu;
      break;

      case 32799: 
      couleur7=violet;
      break;
      }
  switch (vaCOLOR_LED8get) { // 
      case 2016: 
      couleur8=vert;
      break;

      case 63488: 
      couleur8=rouge;
      break;

      case 64832: 
      couleur8=jaune;
      break;

      case 31: 
      couleur8=bleu;
      break;

      case 32799: 
      couleur8=violet;
      break;
      }
  switch (vaCOLOR_LED9get) { // 
      case 2016: 
      couleur9=vert;
      break;

      case 63488: 
      couleur9=rouge;
      break;

      case 64832: 
      couleur9=jaune;
      break;

      case 31: 
      couleur9=bleu;
      break;

      case 32799: 
      couleur9=violet;
      break;
      }
  switch (vaCOLOR_LED10get) { // 
      case 2016: 
      couleur10=vert;
      break;

      case 63488: 
      couleur10=rouge;
      break;

      case 64832: 
      couleur10=jaune;
      break;

      case 31: 
      couleur10=bleu;
      break;

      case 32799: 
      couleur10=violet;
      break;
      }
  switch (vaCOLOR_LED11get) { // 
      case 2016: 
      couleur11=vert;
      break;

      case 63488: 
      couleur11=rouge;
      break;

      case 64832: 
      couleur11=jaune;
      break;

      case 31: 
      couleur11=bleu;
      break;

      case 32799: 
      couleur11=violet;
      break;
      }


      
}
      
//***********************************************************************************************
void shift() {
                    //if (RPM>=4000 && RPM<=4500) {pixels.setPixelColor(0, pixels.Color(255,0,0)) ; }// Moderately bright 0 color.
                    //else {pixels.setPixelColor(0, pixels.Color(0,0,0)) ; }
                    //if (RPM>=4500 && RPM<=50000) {pixels.setPixelColor(1, pixels.Color(255,0,0)) ; }// Moderately bright 0 color.
                    //else {pixels.setPixelColor(1, pixels.Color(0,0,0)) ; }
                    
                  //if (RPM>=0 && RPM<=4000) {pixels.setPixelColor(0, pixels.Color(0,0,0)) ; }// Moderately bright 0 color.



                  //if (RPM>=4000) {pixels.setPixelColor(0, pixels.Color(255,0,0)) ; }// Moderately bright green color.
                  if (RPM>=vaN_LED1get) {pixels.setPixelColor(0, couleur1) ; }// Moderately bright green color.                  
                  else {pixels.setPixelColor(0, pixels.Color(0,0,0)) ; }
                  //pixels.setPixelColor(0, (rpm>=ms_lue_led1  ? pixels.Color(0,255,0) : pixels.Color(0,0,0))); // Moderately bright green color.

                  if (RPM>=vaN_LED2get) {pixels.setPixelColor(1, couleur2) ; }// Moderately bright green color.
                  else {pixels.setPixelColor(1, pixels.Color(0,0,0)) ; } 
                  //pixels.setPixelColor(1, (rpm >= ms_lue_led2  ? pixels.Color(0,255,0) : pixels.Color(0,0,0))); // Moderately bright green color.

                  if (RPM>=vaN_LED3get) {pixels.setPixelColor(2, couleur3) ; }// Moderately bright green color.
                  else {pixels.setPixelColor(2, pixels.Color(0,0,0)) ; }
                  //pixels.setPixelColor(2, (rpm >= ms_lue_led3  ? pixels.Color(0,255,0) : pixels.Color(0,0,0))); // Moderately bright green color.

                  if (RPM>=vaN_LED4get) {pixels.setPixelColor(3, couleur4) ; }// Moderately bright green color.
                  else {pixels.setPixelColor(3, pixels.Color(0,0,0)) ; }
                  //pixels.setPixelColor(3, (rpm >= ms_lue_led4  ? pixels.Color(0,255,0) : pixels.Color(0,0,0))); // Moderately bright green color.                 

                  if (RPM>=vaN_LED5get) {pixels.setPixelColor(4, couleur5) ; }// Moderately bright green color.
                  else {pixels.setPixelColor(4, pixels.Color(0,0,0)) ; }
                  //pixels.setPixelColor(4, (rpm >= ms_lue_led5  ? pixels.Color(0,255,0) : pixels.Color(0,0,0))); // Moderately bright green color.

                  if (RPM>=vaN_LED6get) {pixels.setPixelColor(5, couleur6) ; }// Moderately bright VERT color.
                  else {pixels.setPixelColor(5, pixels.Color(0,0,0)) ; }
                  //pixels.setPixelColor(5, (rpm >= ms_lue_led6  ? pixels.Color(0,255,0) : pixels.Color(0,0,0))); // Moderately bright green color.

                  if (RPM>=vaN_LED7get) {pixels.setPixelColor(6, couleur7) ; }// Moderately bright orange color.
                  else {pixels.setPixelColor(6, pixels.Color(0,0,0)) ; }
                  //pixels.setPixelColor(6, (rpm >= ms_lue_led7  ? pixels.Color(255,127,0) : pixels.Color(0,0,0))); // Moderately bright jaune color.

                  if (RPM>=vaN_LED8get) {pixels.setPixelColor(7,  couleur8) ; }// Moderately bright rouge color.
                  else {pixels.setPixelColor(7, pixels.Color(0,0,0)) ; }
                  //pixels.setPixelColor(7, (rpm >= ms_lue_led8  ? pixels.Color(255,0,0) : pixels.Color(0,0,0))); // Moderately bright rouge color.

                  if  (RPM>=vaN_LED9get) {cligo_Nmax();}

                  if (MAT>=vaN_LED10get) {pixels.setPixelColor(8,  couleur10) ; }// Moderately bright rouge color.
                  else {pixels.setPixelColor(8, pixels.Color(0,0,0)) ; }

                  if (CLT>=vaN_LED11get) {pixels.setPixelColor(9,  couleur11) ; }// Moderately bright rouge color.
                  else {pixels.setPixelColor(9, pixels.Color(0,0,0)) ; }
//pixels.setBrightness(128);
pixels.show();
//delay(10);
}
//*********************************************************************************************************/
