#include <stdio.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/supervisor.h>
#include <math.h> // Para fabs()

#define TIME_STEP 256
#define QtddSensoresProx 8
#define QtddLeds 10
#define QtddCaixa 18

// Tivemos que mudar o DELTA para 0.002, pois o robo msm movendo minimamente a caixa, ja a reconhece como a mais leve, e comeca a girar no proprio eixo
#define DELTA 0.002 

int main(int argc, char **argv) {
  int i, j;
  char texto[256];
  double LeituraSensorProx[QtddSensoresProx];
  double AceleradorDireito = 1.0, AceleradorEsquerdo = 1.0;
  bool caixaMovida = false;

  wb_robot_init();

  WbDeviceTag MotorEsquerdo = wb_robot_get_device("left wheel motor");
  WbDeviceTag MotorDireito = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(MotorEsquerdo, INFINITY);
  wb_motor_set_position(MotorDireito, INFINITY);
  wb_motor_set_velocity(MotorEsquerdo, 0);
  wb_motor_set_velocity(MotorDireito, 0);

  WbDeviceTag SensorProx[QtddSensoresProx];
  for (i = 0; i < QtddSensoresProx; i++) {
    char nome[4];
    sprintf(nome, "ps%d", i);
    SensorProx[i] = wb_robot_get_device(nome);
    wb_distance_sensor_enable(SensorProx[i], TIME_STEP);
  }

  WbDeviceTag Leds[QtddLeds];
  Leds[0] = wb_robot_get_device("led0");
  wb_led_set(Leds[0], -1);

  WbNodeRef caixa[QtddCaixa];
  double posicoesIniciais[QtddCaixa][3];
  for (i = 0; i < QtddCaixa; i++) {
    char nomeCaixa[10];
    sprintf(nomeCaixa, "CAIXA%02d", i);
    caixa[i] = wb_supervisor_node_get_from_def(nomeCaixa);
    if (caixa[i] == NULL)
      printf("Falha ao carregar a posição da %s\n", nomeCaixa);
    else {
      const double *pos = wb_supervisor_node_get_position(caixa[i]);
      for (j = 0; j < 3; j++)
        posicoesIniciais[i][j] = pos[j];
    }
  }

  printf("CAIXAS OK\n\n");

  while (wb_robot_step(TIME_STEP) != -1) {
    for (i = 0; i < 256; i++) texto[i] = 0;

    if (!caixaMovida) {
      for (i = 1; i < QtddCaixa; i++) {
        if (caixa[i] != NULL) {
          const double *posAtual = wb_supervisor_node_get_position(caixa[i]);
          for (j = 0; j < 3; j++) {
            if (fabs(posAtual[j] - posicoesIniciais[i][j]) > DELTA) {
              caixaMovida = true;
              printf("CAIXA%02d FOI MOVIDA!\n", i + 1);
              break;
            }
          }
          if (caixaMovida) break;
        }
      }
    }

    for (i = 0; i < QtddSensoresProx; i++) {
      LeituraSensorProx[i] = wb_distance_sensor_get_value(SensorProx[i]) - 60;
      sprintf(texto, "%s|%d: %5.2f  ", texto, i, LeituraSensorProx[i]);
    }
    printf("%s\n", texto);

    wb_led_set(Leds[0], wb_led_get(Leds[0]) * -1);

    if (caixaMovida) {
      AceleradorDireito = -1;
      AceleradorEsquerdo = 1;
    } else {

      if (LeituraSensorProx[0] > 15 || LeituraSensorProx[7] > 15 || LeituraSensorProx[6] > 100) {
        AceleradorDireito = -1;
        AceleradorEsquerdo = 1;
      } else {
        AceleradorDireito = 1;
        AceleradorEsquerdo = 1;
      }
    }

    wb_motor_set_velocity(MotorEsquerdo, 6.28 * AceleradorEsquerdo);
    wb_motor_set_velocity(MotorDireito, 6.28 * AceleradorDireito);

    printf("POSITIONS: X       Y       Z\n");
    for (i = 0; i < QtddCaixa; i++) {
      if (caixa[i] != NULL) {
        const double *pos = wb_supervisor_node_get_position(caixa[i]);
        printf("CAIXA%02d: %5.2f  %5.2f  %5.2f\n", i + 1, pos[0], pos[1], pos[2]);
      }
    }
    printf("\n");
  }

  wb_robot_cleanup();
  return 0;
}
