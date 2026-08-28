// =====================================================
// CONTROLE DE 5 MOTORES - XIAO ESP32-S3
// Controle individual via Monitor Serial
// =====================================================


// =====================================================
// PINOS DOS MOTORES
// =====================================================

#define MOTOR1 1
#define MOTOR2 3
#define MOTOR3 4
#define MOTOR4 9
#define MOTOR5 7


// =====================================================
// CONFIGURAÇÃO PWM
// =====================================================

#define PWM_FREQ 20000
#define PWM_RESOLUTION 8


// =====================================================
// SETUP
// =====================================================

void setup() {

  Serial.begin(115200);

  delay(1000);

  // ---------------------------------------------------
  // Configura PWM nos GPIOs
  // ---------------------------------------------------

  ledcAttach(MOTOR1, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(MOTOR2, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(MOTOR3, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(MOTOR4, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(MOTOR5, PWM_FREQ, PWM_RESOLUTION);


  // ---------------------------------------------------
  // Garante que todos os motores começam desligados
  // ---------------------------------------------------

  pararTodos();


  // ---------------------------------------------------
  // Mensagem inicial
  // ---------------------------------------------------

  Serial.println();
  Serial.println("========================================");
  Serial.println("      CONTROLE DE 5 MOTORES");
  Serial.println("          XIAO ESP32-S3");
  Serial.println("========================================");

  Serial.println();

  Serial.println("Motores:");

  Serial.println("Motor 1 -> GPIO 1");
  Serial.println("Motor 2 -> GPIO 3");
  Serial.println("Motor 3 -> GPIO 4");
  Serial.println("Motor 4 -> GPIO 9");
  Serial.println("Motor 5 -> GPIO 7");

  Serial.println();

  Serial.println("COMANDOS:");
  Serial.println();

  Serial.println("1 30   -> Motor 1 em 30%");
  Serial.println("2 50   -> Motor 2 em 50%");
  Serial.println("3 70   -> Motor 3 em 70%");
  Serial.println("4 40   -> Motor 4 em 40%");
  Serial.println("5 20   -> Motor 5 em 20%");

  Serial.println();

  Serial.println("Exemplo:");
  Serial.println("1 50");

  Serial.println();

  Serial.println("Para desligar um motor:");
  Serial.println("3 0");

  Serial.println();

  Serial.println("Para desligar TODOS:");
  Serial.println("0");

  Serial.println();

  Serial.println("========================================");
  Serial.println("Aguardando comandos...");
  Serial.println("========================================");
}


// =====================================================
// LOOP
// =====================================================

void loop() {

  // Verifica se chegou algum comando pela serial

  if (Serial.available() > 0) {

    String comando = Serial.readStringUntil('\n');

    comando.trim();


    // -------------------------------------------------
    // Ignora comando vazio
    // -------------------------------------------------

    if (comando.length() == 0) {
      return;
    }


    // -------------------------------------------------
    // COMANDO 0 = DESLIGA TODOS
    // -------------------------------------------------

    if (comando == "0") {

      pararTodos();

      Serial.println();
      Serial.println("TODOS OS MOTORES DESLIGADOS");
      Serial.println();

      return;
    }


    // -------------------------------------------------
    // Procura o espaço entre motor e potência
    // -------------------------------------------------

    int espaco = comando.indexOf(' ');


    if (espaco == -1) {

      Serial.println();
      Serial.println("ERRO!");

      Serial.println("Formato correto:");

      Serial.println("MOTOR POTENCIA");

      Serial.println();

      Serial.println("Exemplo:");

      Serial.println("2 50");

      Serial.println();

      return;
    }


    // -------------------------------------------------
    // Lê número do motor
    // -------------------------------------------------

    int motor =
      comando.substring(0, espaco).toInt();


    // -------------------------------------------------
    // Lê potência
    // -------------------------------------------------

    int potencia =
      comando.substring(espaco + 1).toInt();


    // -------------------------------------------------
    // Verifica número do motor
    // -------------------------------------------------

    if (motor < 1 || motor > 5) {

      Serial.println();

      Serial.println("ERRO!");

      Serial.println("Motor deve ser de 1 a 5.");

      Serial.println();

      return;
    }


    // -------------------------------------------------
    // Verifica potência
    // -------------------------------------------------

    if (potencia < 0 || potencia > 100) {

      Serial.println();

      Serial.println("ERRO!");

      Serial.println("Potencia deve estar entre 0 e 100%.");

      Serial.println();

      return;
    }


    // -------------------------------------------------
    // Aciona o motor
    // -------------------------------------------------

    acionarMotor(motor, potencia);
  }
}


// =====================================================
// FUNÇÃO PARA ACIONAR UM MOTOR
// =====================================================

void acionarMotor(int motor, int potencia) {

  // ---------------------------------------------------
  // Converte 0-100% para PWM 0-255
  // ---------------------------------------------------

  int pwm = map(
    potencia,
    0,
    100,
    0,
    255
  );


  // ---------------------------------------------------
  // Motor 1
  // ---------------------------------------------------

  if (motor == 1) {

    ledcWrite(MOTOR1, pwm);
  }


  // ---------------------------------------------------
  // Motor 2
  // ---------------------------------------------------

  else if (motor == 2) {

    ledcWrite(MOTOR2, pwm);
  }


  // ---------------------------------------------------
  // Motor 3
  // ---------------------------------------------------

  else if (motor == 3) {

    ledcWrite(MOTOR3, pwm);
  }


  // ---------------------------------------------------
  // Motor 4
  // ---------------------------------------------------

  else if (motor == 4) {

    ledcWrite(MOTOR4, pwm);
  }


  // ---------------------------------------------------
  // Motor 5
  // ---------------------------------------------------

  else if (motor == 5) {

    ledcWrite(MOTOR5, pwm);
  }


  // ---------------------------------------------------
  // Mostra informação no Serial
  // ---------------------------------------------------

  Serial.println();

  Serial.print("Motor ");
  Serial.print(motor);

  Serial.print(" -> ");

  Serial.print(potencia);

  Serial.print("%");

  Serial.print(" | PWM = ");

  Serial.println(pwm);

  Serial.println();
}


// =====================================================
// DESLIGA TODOS OS MOTORES
// =====================================================

void pararTodos() {

  ledcWrite(MOTOR1, 0);

  ledcWrite(MOTOR2, 0);

  ledcWrite(MOTOR3, 0);

  ledcWrite(MOTOR4, 0);

  ledcWrite(MOTOR5, 0);
}