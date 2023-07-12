  //this code is made for distance protection for the project done in faculty of engineering menofia 2022-2023 final year
  // students supervised by professur Nagy El-Kalashy for their graduation project
  // here I include the wdt task for the esp in order to remove the idle supervision which resets the code if the idle task isnot reached  
  #include <esp_task_wdt.h>
  // I included the cmath library which is neede for the pow() and abs() operater
  #include <cmath>
  #define i1 Complex(0,1) // this defines the imaginary number i if usage was needed
  #include <LiquidCrystal_I2C.h>
  #include<wire.h>
  #define RESET_PIN 23
  #define TRIP_PIN 18  
  #define LED_PIN 18    
  //LCD connection  
  LiquidCrystal_I2C lcd(0x27,16,2);
  struct Complex{ // this struct defines the new type Complex which is needed for the system
  public: 
  float re; // defines the real part
  float im; // defines the imaginary part`
  Complex(float r=0,float i=0){ // this function allows for quick defining of complex number ex Complex k=Complex(1,2)
    re=r;
    im=i;
  }
  friend Complex operator-(Complex a, Complex b) { //here I override the minus operator to work on complex numbers
    return Complex(a.re - b.re, a.im - b.im);
  }
  friend Complex operator/(Complex a, Complex b) { //here I override the / operator to work on complex number
    float denom = b.re * b.re + b.im * b.im; // denominator
    return Complex((a.re * b.re + a.im * b.im) / denom,
                  (a.im * b.re - a.re * b.im) / denom);
    }
  friend Complex operator+(Complex a, Complex b) { //here i override the plus operator
    return Complex(a.re + b.re, a.im + b.im);
  }
  friend Complex operator*(Complex a, Complex b) { // here i override the multiplication operator
    return Complex(a.re * b.re - a.im * b.im,
                   a.re * b.im + a.im * b.re);
  }
  friend float abs(Complex z) { //here i override thr abs operator
    return sqrt(z.re * z.re + z.im * z.im);
  }
  friend Complex pow(Complex z, int n) { //here i override the pow operator
     if (n == 0) { // zero exponent case
       return Complex(1.0); 
     }
     else { 
       Complex result = z; 
       for (int i = 1; i < n; i++) { 
         result = result * z; 
       }
       return result;
     }
   }
};
float real(Complex z){ //here a function made to return the real part despite not being needed because i already have it in the class above this 
//one is made to look like the matlab one allowing me to copy the code i made in matlab to work in the simulated model here without the need of heavlly editting it
  return z.re;
}
float imag(Complex z){ //here a function made to return the imaginary part despite not being needed because i already have it in the class above this 
//one is made to look like the matlab one allowing me to copy the code i made in matlab to work in the simulated model here without the need of heavlly editting it
  return z.im;
}
float angle(Complex z){ // ths function returns the complex angle in radian 
  return atan2(z.im,z.re);
}
int minNonZeroIndex(float arr[], int n) { // this function returns the minmum non zero index of anarry
  int min = INT_MAX; // here I establihed the current minmum value as the max value an integer can hold
  int index = -1; // here i establish the index as -1 so if the value returned as -1 all elements are zero elements
  for (int i = 0; i < n; i++) { // this loop checks the elements 
    if (arr[i] < min && arr[i] != 0) { // if the element is not zero and is smaller the index is set to it and the current minmum value is set to its value
      min = arr[i];
      index = i;
    }
  }
  return index;
}
 // this section is where i declare the global variables
  const int n=8; // this is the number of samples per cycle
  int halt_fault=1;int wait=0;
  float Y1,Y2,Y3,Y4,Y5,Y6,Y7,Y8; // those are the absloute values of V1,V2,V3,I1,I2,I3
  float w1[n]={0,0,0,0,0,0,0,0}; // this is the window of Va
  float w2[n]={0,0,0,0,0,0,0,0}; // this is the window of Vb
  float w3[n]={0,0,0,0,0,0,0,0}; // this is the window of Vc
  float w4[n]={0,0,0,0,0,0,0,0}; // this is the window of Ia
  float w5[n]={0,0,0,0,0,0,0,0}; // this is the window of Ib
  float w6[n]={0,0,0,0,0,0,0,0}; // this is the window of Ic
  float c[n],s[n]; // here I declare the arrays for the cosine and sine element used i n the DFT
  float k=2*3.1415926535/n; // this is 2*pi/n the dtheta, the change in angle between each sample
  float Vain,Vbin,Vcin;// this is the input for Va,Vb,Vc 
  Complex I00; //Zero Sequence current
  Complex I11; //positive sequence current
  Complex I22; //Negative sequence current
  Complex a120=Complex(-0.5,sqrt(3)/2); // this is a varible used in sequence componnet equation 
  float Iain,Ibin,Icin; // input current signal
  Complex Va,Vb,Vc,Ia,Ib,Ic,Z1ab,Z1bc,Z1ca,Z1a,Z1b,Z1c; // DFT output (Va,Vb,Vc,Ia,Ib,IC), and distance positive sequence impedences (Z1a,Z1b,Z1c,Z1ab,Z1bc,Z1ca) 
  TaskHandle_t DFT; // this is the task handler for FreeRTOS main function whch has both the DFT and the code
  TaskHandle_t Output; // this is the task handler for freeRTOS output task which is mainly used for testing and output
  long Time,fault_time; // this is the time of the operation and the time sice the fault used for the dft
  int fault_type; // this is a number which indicats the fault type
void setup() {
  // initialize LCD
  lcd.init();
  // turn on LCD backlight                      
  lcd.backlight();  
    // set up the LCD's number of columns and rows:
  // Print a message to the LCD.
  lcd.setCursor(0,0);
  lcd.print("system ok");
  Serial.begin(115200); // Here I intiallize the serial connection used in printing 
  pinMode(TRIP_PIN,OUTPUT); // Here I intiallize Pin 18 as an output pin to besed for the tripping signal of the relays
  pinMode(LED_PIN,OUTPUT); // here I intiallize PIN 20 to be used for an indicator fo the tripping signal
  pinMode(RESET_PIN,INPUT);  // Here I intiallize PIN 17 to be used as input pin to deenergize the fault  
  digitalWrite(TRIP_PIN,HIGH); // here I make sure that pin 18 is low
  digitalWrite(LED_PIN,HIGH); // here i makwe sure that pin 20 is low
  for (int i=0;i<=n-1;i++){ // here I set the values for Cosine and sine so I don't need to calculate them each ccle 
    c[i]=2*cos(i*k)/n;
    s[i]=2*sin(i*k)/n;    
  }
  xTaskCreatePinnedToCore( //Here I create the task for the DFT and distance function and pin It to core 0
                    DFTCode,   /* Task function. */
                    "DFT",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &DFT,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */
                    delay(500); 
  xTaskCreatePinnedToCore( //Here I create the task for the output and PIN to core 1
                    OutputCode,   /* Task function. */
                    "Output",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Output,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 0 */ 
  delay(500); 
}
void DFTCode(void * pvParameters){ //thi the main task where both DFt and distance functions is used
  long int  xLastWakeTime1=micros(); // intiaiase task starting time
  long int  xLastWakeTime2; // declare the ending time variable
  int A1; // analouge input int for V1
  int A2; // anlouge input int for V2
  int A3; // analouge input int for V3
  int A4; // analouge input int for I1
  int A5; // anlouge input int for I2
  int A6; // anlouge input int for I3
  int index; //this the fault index
  float fault_location[20]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; //intiallise the fault location array
  double l0,l1,Iaa,Ibb; // intiallise the variables for zero sequence inductance and positive sequence inductance
  int indicator=0; // this is the variable which stores the fault type
  Complex z0,z1,M,Iax,Ibx,Icx,MI00,Zto,Zto1,Zto2,Zto3; // here I intialise basic variable used as parameters for the equation
  // Calaculating line parameters
  l0=0.035/100; // inductance of zero sequence
  l1=0.035/100; // inductance of positive sequence
  z0=(7.8/100+i1*12/100); // +ve sequence impedence
  z1=(7.8/100+i1*12/100); // -ve sequence impedence
  M=(z0-z1)/z1; // M used in distance protection
  Zto=z1*100; // total impedence
  Zto1=0.7*Zto; // zone 1 impedence
  Zto2=1.2*Zto; // zone 2 impedence
  Zto3=2.2*Zto; // zone 3 impedence
  // end of line parameterrs cala  
  esp_task_wdt_init(3000,false);   // disabling wdt reset capabilities as I am not going to allow the controller into the idle task again to improve real time response
  run: // a label to restart the system after a fault is cleared and the system is manually reset through the reset PIN
  digitalWrite(TRIP_PIN,HIGH); // here I make sure the trip signal is low 
  digitalWrite(19,HIGH); // her I make sure the trip signal is low  
  while(1)
  {
    // take analouge reading which ranges from 0-4096
    A1=analogRead(34); 
    A2=analogRead(32);
    A3=analogRead(33);
    A4=analogRead(25);
    A5=analogRead(26);
    A6=analogRead(27);
    // END OF reading analouge
    // callibration of the system
    Vain=10.5/230*A1;
    Vbin=11.5/230*A2;
    Vcin=11.5/230*A3;
    Iain=0.63*A4/95;
    Ibin=0.59*A5/150;
    Icin=0.6*A6/90;
    //end of calibration of the system
    //Zeroing the output voltage before the start of the DFT
    Va.re=0;Va.im=0;Vb.re=0;Vb.im=0;Vc.re=0;Vc.im=0;
    Ia.re=0;Ia.im=0;Ib.re=0;Ib.im=0;Ic.re=0;Ic.im=0;
    // end of the zerroing stage
    // start of the DFT
    for (int i=0;i<=n-2;i++){
      w1[i]=w1[i+1];w2[i]=w2[i+1];w3[i]=w3[i+1]; //applying the concept of sliding window
      w4[i]=w4[i+1];w5[i]=w5[i+1];w6[i]=w6[i+1]; // applying the concept of sliding window
      // start of the main dft equations
      Va.re=Va.re+c[i]*w1[i];
      Va.im=Va.im-s[i]*w1[i];
      Vb.im=Vb.im-s[i]*w2[i];      
      Vb.re=Vb.re+c[i]*w2[i];
      Vc.im=Vc.im-s[i]*w3[i];
      Vc.re=Vc.re+c[i]*w3[i];
      Ia.re=Ia.re+c[i]*w4[i];
      Ia.im=Ia.im-s[i]*w4[i];
      Ib.im=Ib.im-s[i]*w5[i];      
      Ib.re=Ib.re+c[i]*w5[i];
      Ic.im=Ic.im-s[i]*w6[i];
      Ic.re=Ic.re+c[i]*w6[i];
  
    }
    w1[n-1]=Vain; // adding the final value to the end of the window
    w2[n-1]=Vbin; // adding the final value to the end of the window
    w3[n-1]=Vcin; // adding the final value to the end of the window
    w4[n-1]=Iain; // adding the final value to the end of the window
    w5[n-1]=Ibin; // adding the final value to the end of the window
    w6[n-1]=Icin; // adding the final value to the end of the window
    Va.re=Va.re+c[n-1]*w1[n-1];
    Va.im=Va.im-s[n-1]*w1[n-1];
    Vb.re=Vb.re+c[n-1]*w2[n-1];
    Vb.im=Vb.im-s[n-1]*w2[n-1];
    Vc.re=Vc.re+c[n-1]*w3[n-1];
    Vc.im=Vc.im-s[n-1]*w3[n-1];
    Ia.re=Ia.re+c[n-1]*w4[n-1];
    Ia.im=Ia.im-s[n-1]*w4[n-1];
    Ib.re=Ib.re+c[n-1]*w5[n-1];
    Ib.im=Ib.im-s[n-1]*w5[n-1];
    Ic.re=Ic.re+c[n-1]*w6[n-1];
    Ic.im=Ic.im-s[n-1]*w6[n-1];
    // end of the DFT
    // gettting abslout values for testing and printing purposes only, not used in distance protection
    Y1=sqrt(Va.re*Va.re+Va.im*Va.im); // gettin the absloute value of voltage a
    Y2=sqrt(Vb.re*Vb.re+Vb.im*Vb.im); // gettin the absloute value of voltage b
    Y3=sqrt(Vc.re*Vc.re+Vc.im*Vc.im); // gettin the absloute value of voltage c
    Y4=abs(Ia); // gettin the absloute value of current a
    Y5=abs(Ib); // gettin the absloute value of current b
    Y6=abs(Ic); // gettin the absloute value of current c
    // end of data collecting
    // getting phase sequence
    I00=(Va+Vb+Vc)/3.0; // zero phase sequence
    I11=(Va+Vb*a120+Vb*a120*a120)/3.0; // positive phase sequence
    I22=(Va+Vb*a120*a120+Vb*a120)/3.0; // negative phase sequence
    // end of getting phase sequence
    // distance calculations
    Z1ab=0.71*((Va-Vb)/(Ib-Ia)); 
    Z1bc=0.715*((Vb-Vc)/(-1*(Ic)-Ib));
    Z1ca=0.785*((Vc-Va)/(Ia+Ic));
    MI00=M*I00;
    Iax=Ia;
    Ibx=Ib; 
    Icx=Ic+MI00;
    Z1a=1*Va/Iax;
    Z1b=1*Vb/Ibx;
    Z1c=0.9*Vc/Icx;
    

    // end of calculating distance protection
    for (int kk = 0; kk < 19; kk++) {// resetting the fault location array
    fault_location[kk] = 0;
    }
    
    // start of checkking fault zone
    

    

    if (abs(Z1a)<=0.8*abs(Zto1)*abs(cos(3.14/4-angle(Z1a)))){
      fault_location[1]=abs(Z1a)/abs(z1);  
    }else if (abs(Z1a)<=1.5*abs(Zto2)*abs(cos(3.14/4-angle(Z1a)))){
      fault_location[2]=abs(Z1a)/abs(z1);     
    }else if (abs(Z1a)<=abs(Zto3)*abs(cos(3.14/4-angle(Z1a)))){
      fault_location[3]=abs(Z1a)/abs(z1);     
    }
    if (abs(Z1b)<=0.932*abs(Zto1)*abs(cos(3.14/4-angle(Z1b)))){
      fault_location[4]=abs(Z1b)/abs(z1);  
    }else if (abs(Z1b)<=1.2*(abs(Zto2)*abs(cos(3.14/4-angle(Z1b))))){
      fault_location[5]=abs(Z1b)/abs(z1);     
    }else if (abs(Z1b)<=abs(Zto3)*abs(cos(3.14/4-angle(Z1b)))){
      fault_location[6]=abs(Z1b)/abs(z1);     
    }
    if (abs(Z1c)<=0.805*abs(Zto1)){
      fault_location[7]=abs(Z1c)/abs(z1);  
    }else if (abs(Z1c)<=abs(Zto2)){
      fault_location[8]=abs(Z1c)/abs(z1);     
    }else if (abs(Z1c)<=abs(Zto3)){
      fault_location[9]=abs(Z1c)/abs(z1);     
    }
    if (abs(Z1ab)<=0.735*abs(Zto1)*abs(cos(3.14/4-angle(Z1ab)))){
      fault_location[10]=abs(Z1ab)/abs(z1);  
    }else if (abs(Z1ab)<=abs(Zto2)*abs(cos(3.14/4-angle(Z1ab)))){
      fault_location[11]=abs(Z1ab)/abs(z1);     
    }else if (abs(Z1ab)<=abs(Zto3)*abs(cos(3.14/4-angle(Z1ab)))){
      fault_location[12]=abs(Z1ab)/abs(z1);     
    }
    if (abs(Z1bc)<=0.85*abs(Zto1)*abs(cos(3.14/4-angle(Z1bc)))){
      fault_location[13]=abs(Z1bc)/abs(z1);  
    }else if (abs(Z1bc)<=abs(Zto2)*abs(cos(3.14/4-angle(Z1bc)))){
      fault_location[14]=abs(Z1bc)/abs(z1);     
    }else if (abs(Z1bc)<=abs(Zto3)*abs(cos(3.14/4-angle(Z1bc)))){
      fault_location[15]=abs(Z1bc)/abs(z1);     
    }if (abs(Z1ca)<=abs(0.489*Zto1)){
       fault_location[16]=abs(Z1ca)/abs(z1);  
    }else if (abs(Z1ca)<=1.2*abs(Zto2)){
      fault_location[17]=abs(Z1ca)/abs(z1);     
    } else if (abs(Z1ca)<=abs(Zto3)){
      fault_location[18]=abs(Z1ca)/abs(z1);     
    }
    
    if (halt_fault==1){
    index=-1;
    int min = 10000;
    for (int i = 1; i < 20; i++) { // this loop checks the elements 
    if (fault_location[i] < min && fault_location[i] != 0) { // if the element is not zero and is smaller the index is set to it and the current minmum value is set to its value 
      min = fault_location[i];
      index = i;
    }
    }
  
    Y7=index;
    if (wait>=16){
    //Y8=abs(Z1c);
    if (index>=1){
      if (index == 1 || index == 4 || index == 10 || index == 16){
        fault_type=index;
        halt_fault=0;
      }
      if(index == 2 || index == 5  || index == 7 || index == 8 || index == 11 || index == 14 || index == 17 || index == 13) {
        if (indicator==0){
        fault_time=millis();
        indicator=1;
        }
        if((millis()-fault_time)>= 400){
          fault_type=index;
          halt_fault=0;
        }
        }
        if(index == 3 || index == 6 || index == 9 || index == 12 || index == 15 || index == 18) {
        if (indicator==0){
        fault_time=millis();
        indicator=1;
        }
        if((millis()-fault_time)>= 3000){
          fault_type=index;
          halt_fault=0;
        }
        
    }}else{
       fault_time=0;
       indicator=0;
    }
    }
    }
    wait=wait+1;
    if (wait>100) wait=100;
    //end of checking fault location
    
    xLastWakeTime2=micros();
    //Y6=xLastWakeTime2-xLastWakeTime1;
    delayMicroseconds((1000000/(50*n))-(xLastWakeTime2-xLastWakeTime1));
    xLastWakeTime1=micros();
    if (digitalRead(RESET_PIN)==HIGH){
      digitalWrite(18,HIGH); // here I make sure the trip signal is low 
      digitalWrite(19,HIGH); // her I make sure the trip signal is low
      halt_fault=1;
      wait=0;      
    }
    Y7=abs(Z1ca);
    //abs(Z1ca)<=abs(Zto3)*abs(cos(3.14/4-angle(Z1ca))
    Y8=abs(Z1a);

  }
}
void OutputCode(void * pvParameters){
  long int  xLastWakeTime3=micros();
  long int  xLastWakeTime4;
  esp_task_wdt_init(3000,false);  
  int clrr=0;
  while(1){
    Serial.print("Ia:");
    Serial.print(Y4);
    Serial.print(",Ib:");
    Serial.print(Y5);
    Serial.print(",Ic:");
    Serial.print(Y6);
    Serial.print(",Va:");
    Serial.print(Y1);
    Serial.print(",Vb:");
    Serial.print(Y2);
    Serial.print(",Vc:");
    Serial.print(Y3);
    Serial.print(",Z1c:");
    Serial.print(Y8);
    Serial.print(",Z1ac:");
    Serial.println(Y7);   
    if (halt_fault==1){
    xLastWakeTime4=micros();
    Time=xLastWakeTime4-xLastWakeTime3;
    yield();    //delay(10000);
    delayMicroseconds((1000000/(50*n))-(100));
    xLastWakeTime3=micros();  
    digitalWrite(TRIP_PIN,HIGH); // here I make sure that pin 18 is low
    digitalWrite(LED_PIN,HIGH); 
    if (digitalRead(RESET_PIN)==HIGH){
      digitalWrite(TRIP_PIN,HIGH); // here I make sure the trip signal is low 
      digitalWrite(LED_PIN,HIGH); // her I make sure the trip signal is low
      lcd.clear();
      lcd.backlight(); 
      lcd.setCursor(0, 0);
      lcd.print("all systems clr"); 
      halt_fault=1;
      wait=0;      
      clrr=0;
    }
    }      
    if (halt_fault==0){
    digitalWrite(TRIP_PIN,LOW); // here I make sure that pin 18 is low
    digitalWrite(LED_PIN,LOW);
    
    if (clrr==0){
    lcd.clear();    
    clrr++;
    lcd.backlight(); 
    lcd.setCursor(0, 0);
    lcd.print("fault at:");
    lcd.print(fault_type);
    if (fault_type==1){    
    lcd.setCursor(0, 1);     
    lcd.print("L-G at A zone 1");
    }
    if (fault_type==2){       
    lcd.setCursor(0, 1);
    lcd.print("L-G at A zone 2");
    }
    if (fault_type==3){       
    lcd.setCursor(0, 1);
    lcd.print("L-G at A zone 3");
    }
    if (fault_type==4){       
    lcd.setCursor(0, 1);
    lcd.print("L-G at B zone 1");
    }
    if (fault_type==5){       
    lcd.setCursor(0, 1);
    lcd.print("L-G at B zone 2");
    }
    if (fault_type==6){       
    lcd.setCursor(0, 1);
    lcd.print("L-G at B zone 3");
    }
    if (fault_type==7){       
    lcd.setCursor(0, 1);
    lcd.print("L-G at C zone 1");
    }
    if (fault_type==8){       
    lcd.setCursor(0, 1);
    lcd.print("L-G at C zone 2");
    }
    if (fault_type==9){       
    lcd.setCursor(0, 1);
    lcd.print("L-G at C zone 3");
    }
    if (fault_type==10){       
    lcd.setCursor(0, 1);
    lcd.print("L-L at AB zone 1");
    }
    if (fault_type==11){       
    lcd.setCursor(0, 1);
    lcd.print("L-L at AB zone 2");
    }
    if (fault_type==12){       
    lcd.setCursor(0, 1);
    lcd.print("L-L at AB zone 3");
    }
    if (fault_type==13){       
    lcd.setCursor(0, 1);
    lcd.print("L-L at BC zone 1");
    }
    if (fault_type==14){       
    lcd.setCursor(0, 1);
    lcd.print("L-L at BC zone 2");
    }
    if (fault_type==15){       
    lcd.setCursor(0, 1);
    lcd.print("L-L at BC zone 3");
    }
    if (fault_type==16){       
    lcd.setCursor(0, 1);
    lcd.print("L-L at AC zone 1");
    }
    if (fault_type==17){       
    lcd.setCursor(0, 1);
    lcd.print("L-L at AC zone 2");
    }
    if (fault_type==18){       
    lcd.setCursor(0, 1);
    lcd.print("L-L at AC zone 3");
    }
    }
    if (digitalRead(RESET_PIN)==HIGH){
      digitalWrite(TRIP_PIN,HIGH); // here I make sure the trip signal is high 
      digitalWrite(LED_PIN,HIGH); // her I make sure the trip signal is high
      halt_fault==1;
      wait=0;
      clrr=0; 
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("all systems clr");     
      digitalWrite(TRIP_PIN,HIGH); // here I make sure the trip signal is high 
      }
      
    }
    delayMicroseconds((1000000/(50*n))-(100));
  }
}
void loop()
{

}
