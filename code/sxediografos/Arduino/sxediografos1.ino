// Σχεδιογράφος-Μηχ.Σχεδίασης σε κάθετο Τοίχο - Πρόγραμμα

#include <TinyStepper_28BYJ_48.h>        // Βιβλιοθήκη για τον βηματικό κινητήρα. Αν δεν υπάρχει αυτή η βιβλιοθήκη, πατήστε Ctrl+Shift+I και αναζητήστε "Stepper_28BYJ_48" στον διαχειριστή βιβλιοθηκών και εγκαταστήστε την.
#include <Servo.h>                      // Βιβλιοθήκη για τον σερβοκινητήρα.
#include <SD.h>                         // Βιβλιοθήκη SD. Αν δεν υπάρχει αυτή η βιβλιοθήκη, πατήστε Ctrl+Shift+I και αναζητήστε "SD" στον διαχειριστή βιβλιοθηκών και εγκαταστήστε την.
#include <U8x8lib.h>                    // Βιβλιοθήκη U8G2. Αν δεν υπάρχει αυτή η βιβλιοθήκη, πατήστε Ctrl+Shift+I και αναζητήστε "u8g2" στον διαχειριστή βιβλιοθηκών και εγκαταστήστε την.

#define STEPS_PER_TURN  (2048)  // Βήματα βηματικού κινητήρα ανά πλήρη περιστροφή. 2048 βήματα για 360 μοίρες.
#define SPOOL_DIAMETER  (35)    // Διάμετρος καρουλιού σε mm
#define SPOOL_CIRC      (SPOOL_DIAMETER * 3.1416)  // Περιφέρεια καρουλιού 35*3.14=109.956
#define TPS             (SPOOL_CIRC / STEPS_PER_TURN)  // Βηματικό βήμα κινητήρα, ελάχιστη ανάλυση, απόσταση που τραβιέται το σχοινί ανά βήμα 0.053689mm
#define step_delay      1   // Χρόνος αναμονής ανά βήμα του βηματικού κινητήρα (μικροδευτερόλεπτα)
#define TPD             300  
 // Χρόνος αναμονής στροφής (χιλιοστά του δευτερολέπτου). Λόγω αδράνειας, η πένα θα συνεχίσει να κινείται, οπότε αναμένουμε να σταματήσει πριν συνεχίσουμε την κίνηση.
// Κατεύθυνση περιστροφής των δύο κινητήρων: 1 για δεξιόστροφη, -1 για αριστερόστροφη.
// Η ρύθμιση της κατεύθυνσης μέσα/έξω μπορεί να αντιστρέψει την εικόνα κάθετα.
#define M1_REEL_OUT     -1     // Ξετύλιγμα σχοινιού
#define M1_REEL_IN      1      // Τύλιγμα σχοινιού
#define M2_REEL_OUT     1      // Ξετύλιγμα σχοινιού
#define M2_REEL_IN      -1     // Τύλιγμα σχοινιού

static long laststep1, laststep2;
// Τρέχον μήκος σχοινιού, καταγράφει τη θέση της πένας

#define X_SEPARATION  400           // Οριζόντια απόσταση μεταξύ των δύο σχοινιών στην κορυφή σε mm
#define LIMXMAX       ( X_SEPARATION*0.5)   // Μέγιστη τιμή άξονα x. Η θέση 0 βρίσκεται στο κέντρο του σχεδιαστηρίου.
#define LIMXMIN       (-X_SEPARATION*0.5)   // Ελάχιστη τιμή άξονα x
// Παράμετροι κάθετης απόστασης: Θετικές τιμές είναι κάτω από το σχεδιαστήριο, θεωρητικά απεριόριστες αν το σχεδιαστήριο είναι αρκετά μεγάλο. Η αρνητική περιοχή είναι πάνω από την πένα (πριν την εκκίνηση).
#define LIMYMAX         (-250)   // Μέγιστη τιμή άξονα y, το κάτω μέρος του σχεδιαστηρίου
#define LIMYMIN         (250)    // Ελάχιστη τιμή άξονα y, το πάνω μέρος του σχεδιαστηρίου. Κάθετη απόσταση από τα σταθερά σημεία των δύο γραμμών προς την πένα. Προσπαθήστε να μετρήσετε και να τοποθετήσετε με ακρίβεια, μεγάλες αποκλίσεις θα προκαλέσουν παραμόρφωση.
                         
        // Η μείωση της τιμής κάνει το σχέδιο πιο λεπτό και μακρύ, η αύξηση της τιμής το κάνει πιο κοντό και πλατύ.

// Παράμετροι γωνίας σερβοκινητήρα ανύψωσης πένας. Οι συγκεκριμένες τιμές εξαρτώνται από τη θέση του βραχίονα και πρέπει να ρυθμιστούν.
#define PEN_UP_ANGLE    85  // Πένα πάνω
#define PEN_DOWN_ANGLE  70  // Πένα κάτω
// Παράμετροι που χρειάζονται ρύθμιση ===
#define PEN_DOWN 1  // Κατάσταση πένας: Κάτω
#define PEN_UP 0    // Κατάσταση πένας: Πάνω

U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE);
struct point { 
  float x; 
  float y; 
  float z; 
};

struct point actuatorPos;

// Θέση plotter (πένας).
static float posx;
static float posy;
static float posz;  // Κατάσταση πένας
static float feed_rate = 0;

// Κατάσταση πένας (πάνω, κάτω).
static int ps;

/* Παράμετροι επικοινωνίας G-code */
#define BAUD            (115200)    // Ρυθμός σειριακής θύρας, χρησιμοποιείται για μεταφορά G-code ή εντοπισμό σφαλμάτων. Επιλογές: 9600, 57600, 115200 ή άλλοι κοινοί ρυθμοί.
#define MAX_BUF         (64)      // Μέγεθος buffer σειριακής θύρας

// Λήψη σειριακής επικοινωνίας
static int sofar;
// Πρόοδος σειριακού buffer

static float mode_scale;   // Κλίμακα
File myFile;

Servo pen;
#define BEEP_SW 1          // Ενεργοποίηση/απενεργοποίηση ηχητικής ειδοποίησης μετά την ολοκλήρωση της εργασίας
#define BEEP A2            // Πιν για τον βομβητή
#define ADKEY A1           // Πιν για το αναλογικό πλήκτρο
#define KEYDOWN 330        // Αναλογική τιμή για το πλήκτρο "Κάτω"
#define KEYENTER 90        // Αναλογική τιμή για το πλήκτρο "Enter"
#define KEYESC 517         // Αναλογική τιμή για το πλήκτρο "Esc"

TinyStepper_28BYJ_48 m1; //(7,8,9,10); // Βηματικός κινητήρας M1 L, οι θύρες in1~4 αντιστοιχούν στις 7, 8, 9, 10 του UNO
TinyStepper_28BYJ_48 m2; //(2,3,5,6);  // Βηματικός κινητήρας M2 R, οι θύρες in1~4 αντιστοιχούν στις 2, 3, 5, 6 του UNO

//------------------------------------------------------------------------------
// Υπολογισμός κίνησης προς τα εμπρός - Μετατροπή μηκών L1, L2 σε συντεταγμένες XY
// Χρησιμοποιώντας το νόμο των συνημιτόνων, theta = acos((a*a+b*b-c*c)/(2*a*b));
// Βρείτε τη γωνία μεταξύ M1M2 και M1P, όπου P είναι η θέση της πένας
void FK(float l1, float l2,float &x,float &y) {
  float a=l1 * TPS;
  float b=X_SEPARATION;
  float c=l2 * TPS;
// Μέθοδος 1
  float theta = acos((a*a+b*b-c*c)/(2.0*a*b));
  x = cos(theta)*l1 + LIMXMIN;
  y = sin(theta)*l1 + LIMYMIN;
// Μέθοδος 2
/* float theta = (a*a+b*b-c*c)/(2.0*a*b);
  x = theta*l1 + LIMXMIN;
y = sqrt (1.0 - theta * theta ) * l1 + LIMYMIN;*/
}


//------------------------------------------------------------------------------
// Αντίστροφη κίνηση - Μετατροπή συντεταγμένων XY σε μήκη L1, L2 
void IK(float x,float y,long &l1, long &l2)
{
  float dy = y - LIMYMIN;
float dx = x - LIMXMIN;
  l1 = round(sqrt(dx*dx+dy*dy) / TPS);
  dx = x - LIMXMAX;
l2 = round(sqrt(dx*dx+dy*dy) / TPS);
}

//------------------------------------------------------------------------------
// Κατάσταση πένας
void pen_state(int pen_st)
{
  if(pen_st==PEN_DOWN)
  {
    ps=PEN_DOWN_ANGLE;
    // Serial.println("Pen down");
} 
  else 
  {
    ps=PEN_UP_ANGLE;
    //Serial.println("Pen up");
  }
  pen.write(ps);
}

void pen_down()
{ 
  if (ps==PEN_UP_ANGLE)
  {
    ps=PEN_DOWN_ANGLE;
    pen.write(ps);
    delay(TPD);
}
}

void pen_up()
{
  if (ps==PEN_DOWN_ANGLE)
  {
    ps=PEN_UP_ANGLE;
    pen.write(ps);
}  
}

//------------------------------------------------------------------------------
// Επιστρέφει τη γωνία του dy/dx ως τιμή από 0...2Π
static float atan3(float dy, float dx) {
  float a = atan2(dy, dx);
if (a < 0) a = (PI * 2.0) + a;
  return a;
}


//------------------------------------------------------------------------------
// Σχεδίαση τόξου
static void arc(float cx, float cy, float x, float y,  float dir) {
  // Λήψη ακτίνας
  float dx = posx - cx;
float dy = posy - cy;
  float radius = sqrt(dx * dx + dy * dy);
// Εύρεση γωνίας τόξου (sweep)
  float angle1 = atan3(dy, dx);
float angle2 = atan3(y - cy, x - cx);
  float theta = angle2 - angle1;
if (dir > 0 && theta < 0) angle2 += 2 * PI;
else if (dir < 0 && theta>0) angle1 += 2 * PI;
// Λήψη μήκους τόξου
  // float circ=PI*2.0*radius;
  // float len=theta*circ/(PI*2.0);
// Απλοποιείται σε
  float len = abs(theta) * radius;

  int i, segments = floor(len / TPS);
float nx, ny, nz, angle3, scale;

  for (i = 0; i < segments; ++i) {

    if (i==0) 
      pen_up();
else
      pen_down();  
    scale = ((float)i) / ((float)segments);

    angle3 = (theta * scale) + angle1;
nx = cx + cos(angle3) * radius;
    ny = cy + sin(angle3) * radius;
// Στείλτε το στον planner
    line_safe(nx, ny);
  }

  line_safe(x, y);
  pen_up();
}



//------------------------------------------------------------------------------
// Άμεση μετακίνηση της εικονικής θέσης του plotter
// Δεν επικυρώνει αν η κίνηση είναι έγκυρη
static void teleport(float x, float y) {
  posx = x;
posy = y;
  long l1,l2;
  IK(posx, posy, l1, l2);
  laststep1 = l1;
  laststep2 = l2;
}


//==========================================================
// Αναφορά — Πρόγραμμα διαγώνιας γραμμής
int workcnt=0;
void moveto(float x,float y) 
{
  long l1,l2;
  IK(x,y,l1,l2);
  long d1 = l1 - laststep1;
long d2 = l2 - laststep2;

  long ad1=abs(d1);
  long ad2=abs(d2);
  int dir1=d1>0 ? M1_REEL_IN : M1_REEL_OUT;
  int dir2=d2>0 ?
M2_REEL_IN : M2_REEL_OUT;
  long over=0;
  long i;


  if(ad1>ad2) {
    for(i=0;i<ad1;++i) {
      
      m1.moveRelativeInSteps(dir1);
over+=ad2;
      if(over>=ad1) {
        over-=ad1;
        m2.moveRelativeInSteps(dir2);
      }
      delayMicroseconds(step_delay);
}
  } 
  else {
    for(i=0;i<ad2;++i) {
      m2.moveRelativeInSteps(dir2);
      over+=ad1;
if(over>=ad2) {
        over-=ad2;
        m1.moveRelativeInSteps(dir1);
      }
      delayMicroseconds(step_delay);
}
  }

  laststep1=l1;
  laststep2=l2;
  posx=x;
  posy=y;
}

//------------------------------------------------------------------------------
// Οι κινήσεις μεγάλης απόστασης θα ακολουθήσουν κυκλική τροχιά, επομένως οι μεγάλες γραμμές κόβονται σε μικρές για να διατηρήσουν ευθεία μορφή.
static void line_safe(float x,float y) {
  // Διαχωρίζετε τις μεγάλες γραμμές για να τις κάνετε πιο ευθείες;
float dx=x-posx;
  float dy=y-posy;

  float len=sqrt(dx*dx+dy*dy);
  
  if(len<=TPS) {
    moveto(x,y);
    return;
  }
  
  // Πολύ μεγάλη!
long pieces=floor(len/TPS);
  float x0=posx;
  float y0=posy;
  float a;
  for(long j=0;j<=pieces;++j) {
    a=(float)j/(float)pieces;
moveto((x-x0)*a+x0,
         (y-y0)*a+y0);
  }
  moveto(x,y);
}


void line(float x,float y) 
{
  line_safe(x,y);
}


//********************************
void nc(String st)
{
  String xx,yy,zz;
  int ok=1;
  st.toUpperCase();
  
  float x,y,z;
  int px,py,pz;
  px = st.indexOf('X');
  py = st.indexOf('Y');
  pz = st.indexOf('Z');
  if (px==-1 || py==-1) ok=0; 
  if (pz==-1) {pz=st.length();}
  else
  {   
    zz = st.substring(pz+1,st.length());
    z  = zz.toFloat();
    if (z>0)  pen_up();
    if (z<=0) pen_down();
  }

  xx = st.substring(px+1,py);
  yy = st.substring(py+1,pz);
  
  xx.trim();// Εσοχή, αφαιρεί τα κενά στο τέλος
  yy.trim();
if (ok) line(xx.toFloat(),yy.toFloat()); 
}
//**********************
void working(void)
{
    u8x8.clear();
    u8x8.draw1x2String(0, 0, "Working"); 
}
//**********************
void drawfile( String filename)
{
  String rd="";
int line=0;
  char rr=0;
  myFile = SD.open(filename);
  if (myFile) 
  {
    working();
while (myFile.available()) {
    rr=myFile.read();
    if (rr == char(10)) 
    {
      line++;
      nc(rd);
      rd="";
}
    else
    {
       rd+=rr;
}  
    }
    myFile.close(); 
  }
  else
  {
    u8x8.clear();
u8x8.drawString(0, 3, "Nofile"); // "No file"
    delay(2000);
  }
}
  // Τοποθετήστε τον κώδικα ρύθμισης εδώ, για να εκτελεστεί μία φορά:
void setup() {
  Serial.begin(BAUD);
  pinMode(BEEP,OUTPUT);
  digitalWrite(BEEP,LOW);
  u8x8.begin();
  u8x8.setFont(u8x8_font_amstrad_cpc_extended_r);
  u8x8.draw1x2String(1, 3, "-- Welcome! --");
  m1.connectToPins(7,8,9,10); // Βηματικός κινητήρας M1 L, οι θύρες in1~4 αντιστοιχούν στις 7, 8, 9, 10 του UNO
  m2.connectToPins(2,3,5,6); // Βηματικός κινητήρας M2 R, οι θύρες in1~4 αντιστοιχούν στις 2, 3, 5, 6 του UNO
  m1.setSpeedInStepsPerSecond(10000);
  m1.setAccelerationInStepsPerSecondPerSecond(100000);
  m2.setSpeedInStepsPerSecond(10000);
  m2.setAccelerationInStepsPerSecondPerSecond(100000);
  pen.attach(A0);   // Σερβοκινητήρας ανύψωσης πένας
  ps=PEN_UP_ANGLE;
  pen.write(ps);
  teleport(0, 0);// Ορίστε την τρέχουσα θέση της πένας σε 0,0
  mode_scale = 1; // Συντελεστής κλίμακας
  //Serial.println(F("Test OK!"));   
  delay(1500);
  u8x8.clear();  
}

int KeyCheck(void)
{
  int value=0;
int n = analogRead(ADKEY);
  if(n<1020)
  {
    if(n<(KEYDOWN+10))
    {
      if(n>(KEYDOWN-10))
      {
        value=1;
}  
    }  
    if(n<(KEYENTER+10))
    {
      if(n>(KEYENTER-10))
      {
        value=2;
}  
    } 
    if(n<(KEYESC+10))
    {
      if(n>(KEYESC-10))
      {
        value=3;
}  
    } 
  }
  else
  {
    value=0;
}
  return value;
}

void beep(void)
{
  int i=5;
  m1.disableMotor();
  m2.disableMotor();
  if (!BEEP_SW) return;
while(i--)
  {
    digitalWrite(BEEP,HIGH);   
    delay(300);    
    digitalWrite(BEEP,LOW);
    delay(300);
  }
}

void loop() {
  int keyvalue=3;
  int maincase=1;
  pen_up();
while(true)
  {    
    if(keyvalue!=0)
    {
      while(KeyCheck());
if(keyvalue==1)// Επιλογή
      {
        //Serial.println((maincase));
if(maincase<10)
        {
          if(maincase<3) maincase++;
else maincase=1;
        }
        else// Σε αυτή την περίπτωση, μόνο για την οθόνη ρυθμίσεων
        {  
          if(maincase==30) maincase=31;
//else if(maincase==31) maincase=32;
          else maincase=30;
        }
      }
      else if(keyvalue==2)// Επιβεβαίωση
      {
        if(maincase==1)
        {
          // Εκτέλεση DEMO  
          working();
my_demo();
          beep();
          u8x8.clear();
        }
        else if(maincase==2)
        {
          // Εκτέλεση SD 
          if (!SD.begin(4)) {
            //Serial.println(F("initialization SD failed!"));
u8x8.clear();
            u8x8.draw1x2String(0, 0, "Err"); // "Σφάλμα"
            delay(2000);
            u8x8.clear();
          }
          else
          {            
            pen_up();
drawfile("main.nc");
            beep();
            u8x8.clear();
          }
        }
        else if(maincase==30)
        {
          moveto(0,251);
}
        else if(maincase==31)
        {
          moveto(0,0);
}
        else
        {
          maincase*=10;
u8x8.clear(); 
        }
      }
      else if(keyvalue==3)// Ακύρωση
      {
        if(maincase>9)
        {
          maincase/=10;
}
        u8x8.clear();  
      }
      switch(maincase)
      {
        case 1:
        {
          u8x8.draw1x2String(0, 5, " ");
u8x8.draw1x2String(0, 1, "- DEMO");
          u8x8.draw1x2String(2, 3, "SD");
          u8x8.draw1x2String(2, 5, "SET"); // "ΡΥΘΜΙΣΕΙΣ"
        }break;
case 2:
        {
          u8x8.draw1x2String(0, 1, " ");
u8x8.draw1x2String(2, 1, "DEMO");
          u8x8.draw1x2String(0, 3, "- SD");
          u8x8.draw1x2String(2, 5, "SET"); // "ΡΥΘΜΙΣΕΙΣ"
        }break;
case 3:
        {
          u8x8.draw1x2String(0, 3, " ");
u8x8.draw1x2String(2, 1, "DEMO");
          u8x8.draw1x2String(2, 3, "SD");
          u8x8.draw1x2String(0, 5, "- SET"); // "- ΡΥΘΜΙΣΕΙΣ"
        }break;
case 30:
        {
          u8x8.draw1x2String(0, 0, "- Top"); // "- Πάνω"
u8x8.draw1x2String(0, 2, "  Zero"); // "  Μηδέν"
        }break;
        case 31:
        {
          u8x8.draw1x2String(0, 0, "  Top"); // "  Πάνω"
u8x8.draw1x2String(0, 2, "- Zero"); // "- Μηδέν"
        }break;
        case 32:
        {
          u8x8.draw1x2String(0, 0, "  Top"); // "  Πάνω"
u8x8.draw1x2String(0, 2, "  Zero"); // "  Μηδέν"
        }break;
        default:break;  
      }
    }
    keyvalue=KeyCheck();
}
}

// Καμπύλη πεταλούδας
void butterfly_curve(int xx,int yy,int lines,int x_scale,int y_scale)
// xx,yy κεντρική θέση πεταλούδας, lines αριθμός γύρων, όσο περισσότεροι τόσο πιο περίπλοκο, x_scale,y_scale συντελεστής μεγέθυνσης άξονα xy
{
  float xa,ya,p,e;
  pen_up();
  moveto(xx,yy + y_scale  * 0.71828);
  pen_down();
  for(float i=0;i<6.28*lines;i+=3.14/90)
  {   
    p=pow(sin(i/12),5);
    e=pow(2.71828,cos(i));
    xa=x_scale * sin(i) * (e - 2*cos(4*i) + p);
    ya=y_scale * cos(i) * (e - 2*cos(4*i) + p);
    line_safe(xa+xx,ya+yy);
  }
  pen_up();
}  

// Καμπύλη καρδιάς
void heart_curve(int xx,int yy,float x_scale,float y_scale)
// xx,yy κεντρική θέση καμπύλης καρδιάς, x_scale,y_scale συντελεστής μεγέθυνσης άξονα xy
{
  float xa,ya;
  
  pen_up();
moveto(xx,yy+y_scale * 7);
  pen_down();
  for(float i=0;i<=6.28;i+=3.14/180)
  {       
    xa=x_scale * pow(sin(i),3) * 15;
ya=y_scale * (15*cos(i) -5*cos(2*i) - 2*cos(3*i) - cos(4*i));
    line_safe(xa+xx,ya+yy);  
  }
  pen_up();
} 

void rectangle(float xx,float yy,float dx,float dy,float angle) // Σχεδίαση Κουτί 1
{
  float six,csx,siy,csy;
  dx/=2;
  dy/=2;
  six = sin(angle/180*3.14) * dx;
  csx = cos(angle/180*3.14) * dx;
  siy = sin(angle/180*3.14) * dy;
  csy = cos(angle/180*3.14) * dy; 
  pen_up();
  line_safe(csx - siy + xx,six + csy + yy);
  pen_down();
  line_safe(xx - csx - siy,csy - six + yy);
  line_safe(xx - csx + siy,yy - csy - six);
  line_safe(csx + siy + xx,six - csy + yy);
  line_safe(csx - siy + xx,six + csy + yy);
  pen_up();
}

void box(float xx,float yy,float dx,float dy) // Σχεδίαση Κουτί 2
  {
  pen_up();
  line_safe(xx , yy);
  pen_down();
  delay(TPD);
  line_safe(xx + dx, yy);
  delay(TPD);
  line_safe(xx + dx, yy+ dy);
  delay(TPD);
  line_safe(xx , yy + dy);
  delay(TPD);
  line_safe(xx , yy);
  pen_up();
}

void circle(float xx,float yy,float radius_x,float radius_y)// Κύκλος
{
  float rx,ry;
  float st= 3.14159 / 90; // Ακρίβεια διαίρεσης κύκλου
  pen_up();
  line(xx+radius_x,yy);
  pen_down();
for(float i=0;i<6.28318;i+=st)
  {
    rx = cos(i) * radius_x;
    ry = sin(i) * radius_y;
    line(xx+rx,yy+ry);
}
  pen_up();
}

void my_demo()
{
  pen_up();
  box(-45,0,90,90);
  moveto(-15,0);
  pen_down();
  line(-15,90);
  pen_up();
  moveto(15,90); 
  pen_down();
  line(15,0);
  pen_up();
  moveto(-45,30); 
  pen_down();
  line(45,30);
  pen_up();
  moveto(45,60); 
  pen_down();
  line(-45,60);
  pen_up();
  box(-42.5,62.5,25,25);
  circle(0,75,12.5,12.5);
  rectangle(30,75,17.7,17.7,45);
  rectangle(-30,45,17.7,17.7,45);
  box(-12.5,32.5,25,25);
  circle(30,45,12.5,12.5);
  circle(-30,15,12.5,12.5);
  rectangle(0,15,17.7,17.7,45);
  box(17.5,2.5,25,25);
  heart_curve(-45,-45,2,2);  // Καμπύλη καρδιάς. Περιγραφή παραμέτρων (x, y θέση, συντελεστής μεγέθυνσης x, συντελεστής μεγέθυνσης y)
  butterfly_curve(45,-55,3,12,12); // Καμπύλη πεταλούδας. Περιγραφή παραμέτρων (x, y θέση, όσο περισσότεροι γύροι τόσο πιο περίπλοκο, συντελεστής μεγέθυνσης x, συντελεστής μεγέθυνσης y)
}