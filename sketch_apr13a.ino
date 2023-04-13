#include "BluetoothSerial.h" //Se incluye la librería de comunicaciones por Bluetooth


BluetoothSerial ESP_BT; //Creación del objeto del tipo BluetoothSerial

//DEFINICIÓN DE VARIABLES Y CONSTANTES

// datos para captura de funcionamiento
#define RECSAMPLES 2000 //Cantidad de muestras que se capturan cuando se introduce el comando de capturar             
#define frec_lim_sup 50
#define intensidad_lim 5.04
#define sqrt2 1.414213562 //definimos la raiz de 2 como constante
#define Vdc 325.269119 //Tensión Vdc a utilizar en las fórmulas
#define AN_PORT_1 36
#define AN_PORT_2 39

//paso de control
float dt = 200e-6; //paso de tiempo entre bucles
unsigned int dtus; //momento en el que nos encontramos


char BT_data[32]=""; //Buffer a leer de lo recibido por Bluetooth (Comandos)
boolean swon, swoff, pwon; //Variables booleanas que implican que se ha activado el control ("swon"), que se ha desactivado ("swoff")
boolean record, recorded; 
short recsamples = RECSAMPLES;
// Vectores que almacenan los datos que se han recogido. Cada vector es del tamaño especificado en la constante RECSAMPLES
short recdata1[RECSAMPLES];
unsigned short recdata2[RECSAMPLES];
unsigned short recdata3[RECSAMPLES];
short rsample = 0; //Variables que se utiliza para indexar los vectores recdata's
short chA=0;
short chB=1;
short chC=2;
short dutyA, dutyB, dutyC; //Valores que irán de 0 a 4095 para determinar cuánto tiempo estará activada cada rama en cada período de conmutación
short freConmut=10000;
short resolucion=12;
const int rama_a=15;
const int rama_b=2;
const int rama_c=4;

float frec_ref=5; //frecuencia de referencia a alcanzar pedida por el usuario
//float t_rampa=0;
float frec_actual=5; //frec_actual: frecuencia leída en el último paso;
float angle=0;
float modulo=0;
short sector;
float Valfa, Vbeta, dk, dkmas1, dnulosmedios, da_A, db_A, dc_A; //Variables del SVM
//constantes
float pitercios, r3, sin60;

float corriente1=0;
float corriente2=0;
float corriente3=0;

float fasor_I_RE, fasor_I_IM, modulo_I, angle_I; 



#define GPIO_TESTCLK 17

#define GPIO_ENABLE 16

TaskHandle_t Task2;

void setup() 
{
  pinMode(GPIO_TESTCLK, OUTPUT);
  pinMode(GPIO_ENABLE, OUTPUT);
  pinMode(AN_PORT_1,INPUT);
  pinMode(AN_PORT_2,INPUT);
  digitalWrite(GPIO_TESTCLK, LOW);
  digitalWrite(GPIO_ENABLE, LOW);

  dtus = dt*1e6;

  Serial.begin(115200);
  // start Bluetooth
  ESP_BT.begin("ESP32Drive_2"); //Name of your Bluetooth Signal
  //create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(
                    Task2code,   /* Task function. */
                    "Task2",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task2,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */
  
  //configuraciÃ³n pines encoder
  pitercios = PI/3;
  sin60=sin(pitercios);
  r3=sqrt(3);

  ledcSetup(chA,freConmut,resolucion);
  ledcAttachPin(rama_a,chA);
  ledcSetup(chB,freConmut,resolucion);
  ledcAttachPin(rama_b,chB);
  ledcSetup(chC,freConmut,resolucion);
  ledcAttachPin(rama_c,chC);

}
void loop() 
{

  if(swon)
  {
    // aquÃ­ va todo el cÃ³digo que se tenga que ejecutar cada vez que se ponga en marcha el control (inicializaciÃ³n)
    pwon = true;
    swon = false;
    digitalWrite(GPIO_ENABLE,HIGH);
    ESP_BT.println("Switched on");
    corriente1 = 0;
    corriente2 = 0;
    frec_ref = 5;     //set reference frequency to 5
    frec_actual = 5;

  }
  
  if(swoff)
  {
    // aquÃ­ va todo el cÃ³digo que se tenga que ejecutar cada vez que se apague el control (finalizaciÃ³n)
    digitalWrite(GPIO_ENABLE,LOW);
    ESP_BT.println("Switched off");
    pwon = false;
    swoff = false;
    angle=0;
    modulo=0;
    
    frec_ref = 5;     //set reference frequency to 5
    frec_actual = 5;

  }

  unsigned int nowus, nextus;
  
  nextus = micros()+dtus;
  
  while(pwon&&!swoff)
  {
    // aquÃ­ va todo el cÃ³digo que se tiene que ejecutar con el paso fijo de dtus microsegundos
   // lo primero de todo, ponemos a nivel alto el pin de estado "calculando"
    digitalWrite(GPIO_TESTCLK,true);

    //SUBSTITUIR CON NUEVAS ECUACIONES
    corriente1=((float)analogRead(AN_PORT_1)-1888.200)*0.00393456107;
    corriente2=((float)analogRead(AN_PORT_2)-1888.600)*0.00391203599;
    corriente3=-corriente1-corriente2;

    fasor_I_RE=0.66666667*(corriente1*1+corriente2*(-0.5)+corriente3*(-0.5));
    fasor_I_IM=0.66666667*(corriente2*0.866025+corriente3*(-0.866025));

    modulo_I=sqrt(pow(fasor_I_RE,2)+pow(fasor_I_IM,2));

    if(modulo_I>=intensidad_lim)
    {
      digitalWrite(GPIO_ENABLE,LOW);
      swoff=true;
    }
    

    if(frec_actual != frec_ref) //25Hz/s
    {
      if(frec_actual<frec_ref)
      {
        frec_actual+=25*dt;
        if(frec_actual>frec_ref) frec_actual=frec_ref;
      }
      else if(frec_actual>frec_ref)
      {
        frec_actual-=25*dt;
        if(frec_actual<frec_ref) frec_actual=frec_ref;
      }
    } 

    //Cálculo del ángulo en radianes
  
    angle=2*PI*frec_actual*dt+angle; 
    if(angle>2*PI) 
      {
        angle=angle-2*PI;
      }
    else if(angle<0) 
    {
      angle=angle+2*PI;
    }

    //Módulo proporcional a la frecuencia 50Hz >> 230*sqrt(2)
    modulo=frec_actual*(Vdc/r3)/50.0;
    //Cálculo del sector a partir del ángulo
    sector=(short)floor(angle/pitercios)+1;
    //Cálculo de Valfa y Vbeta
    Valfa=modulo*cos(angle);
    Vbeta=modulo*sin(angle);
    //Cases para el cálculo de dk, dk+1 y dnulos

    //Select the right formula depending on the sector
    switch (sector) 
    {
      case 1: 
        dk=(r3/Vdc)*(sin60*Valfa-0.5*Vbeta);
        dkmas1=(r3/Vdc)*(Vbeta);
        dnulosmedios=(1.0-dk-dkmas1)/2.0;
        da_A=dk+dkmas1+dnulosmedios; 
        db_A=dkmas1+dnulosmedios;
        dc_A=dnulosmedios;
      break;
     
      case 2: 
        dk=(r3/Vdc)*(sin60*Valfa+0.5*Vbeta);
        dkmas1=(r3/Vdc)*(-sin60*Valfa+0.5*Vbeta);
        dnulosmedios=(1.0-dk-dkmas1)/2.0;
        da_A=dnulosmedios+dk; 
        db_A=dkmas1+dk+dnulosmedios;
        dc_A=dnulosmedios;
      break;
      
      case 3: 
        dk=(r3/Vdc)*(Vbeta);
        dkmas1=(r3/Vdc)*(-sin60*Valfa-0.5*Vbeta);
        dnulosmedios=(1.0-dk-dkmas1)/2.0;
        da_A=dnulosmedios; 
        db_A=dkmas1+dk+dnulosmedios;
        dc_A=dnulosmedios+dkmas1;
      break;
      
      case 4:
        dk=(r3/Vdc)*(-sin60*Valfa+0.5*Vbeta);
        dkmas1=-(r3/Vdc)*Vbeta;
        dnulosmedios=(1.0-dk-dkmas1)/2.0;
        da_A=dnulosmedios; 
        db_A=dk+dnulosmedios;
        dc_A=dkmas1+dk+dnulosmedios;
      break;
      
      case 5:
        dk=(r3/Vdc)*(-sin60*Valfa-0.5*Vbeta);
        dkmas1=(r3/Vdc)*(sin60*Valfa-0.5*Vbeta);
        dnulosmedios=(1.0-dk-dkmas1)/2.0;
        da_A=dkmas1+dnulosmedios; 
        db_A=dnulosmedios;
        dc_A=dk+dkmas1+dnulosmedios;
      break;
  
      case 6: 
        dk=-(r3/Vdc)*Vbeta;
        dkmas1=(r3/Vdc)*(sin60*Valfa+0.5*Vbeta);
        dnulosmedios=(1.0-dk-dkmas1)/2.0;
        da_A=dkmas1+dk+dnulosmedios; 
        db_A=dnulosmedios;
        dc_A=dnulosmedios+dk;
      break;
    }

    //calculo de duty cycle en digital
    dutyA=da_A*4095;
    dutyB=db_A*4095;
    dutyC=dc_A*4095;
    ledcWrite(chA,dutyA);
    ledcWrite(chB,dutyB);
    ledcWrite(chC,dutyC);
 

    if(record)
    {
      recdata1[rsample] = (short)(modulo_I*1000); //se registra los valores de frecuencia en cHz
      recdata2[rsample] = (short)(frec_ref*1000); //pon aquÃ­ los datos que quieras registrar, en lugar de los tres "0"
      recdata3[rsample] = (short)(frec_actual*1000); 
      rsample ++;
      
     if(rsample == recsamples)
      {
        record = false;
        recorded = true;
      }
    }

    // cuando terminamos los cÃ¡lculos, bajamos a nivel bajo el el pin de estado "calculando"
    digitalWrite(GPIO_TESTCLK,false);
    //esperamos mientras el siguiente microsegundo de fin de paso de control (nextus) sea mayor que el microsegundo de estado del micro (micros())
    while(nextus+1 > micros()){}
    //actualizamos el siguiente microsegundo de fin de paso de control
    nextus += dtus;
  }
  
  digitalWrite(GPIO_ENABLE,LOW);
}


void Task2code( void * pvParameters )
{

  short posBT = 0;

  while(true)
  {
    while(ESP_BT.available())
    { //Check if we receive anything from Bluetooth
      BT_data[posBT]=ESP_BT.read();
      posBT++;
      //mostramos por el puerto serie lo que hemos recibido por BlueTooth
      Serial.println(BT_data[posBT]);
    }
    
    if(posBT)
    {
      if(BT_data[0] == 49 )
      {
        swon=true;
      }
      else if(BT_data[0] == 48)
      {
        swoff=true;
      }

      // aquÃ­ podemos incluir tantos "else if(BT_data[0] == ASCII_code)" como queramos para responder a comandos recibidos por BlueTooth que empiecen por ASCII_code
      else if(BT_data[0] == 99) 
      {
        record = true; 
        rsample = 0;
      }
      
      else if(BT_data[0] == 70 || BT_data[0] == 102)
      { //f de frecuencia
        record=true;
        delay(40);
        frec_ref=atof(&BT_data[1]);
        ESP_BT.print(frec_ref);
      }
    }
      
    posBT = 0;

    ESP_BT.print(sector); //imprime el sector
    ESP_BT.print(";");

    ESP_BT.print(swon); //imprime swon
    ESP_BT.print(";");

    ESP_BT.println(";");
    if(recorded) {report();}

    delay(100);
  }
}

void report()
{
  ESP_BT.println("Report start;");
  for(short rsample = 0; rsample < recsamples; rsample++)
  {
    ESP_BT.print(";");

    ESP_BT.print(rsample);
    ESP_BT.print(";");

    ESP_BT.print(recdata1[rsample]);
    ESP_BT.print(";");
    ESP_BT.print(recdata2[rsample]);
    ESP_BT.print(";");
    ESP_BT.print(recdata3[rsample]);
    ESP_BT.println(";");

     
  }
  recorded = false;

}