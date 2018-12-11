//Carrinho seguidor de linha com controlador PID- Cleyson e Kassia



/*
Esse codigo consiste no controle de um carrinho seguidor de linha por 
meio da leitura de uma placa de 5 sensores infraverelho 
presente no carrinho e um controlador PID a apartir de sua leitura
Configuraçao do erro com relacao a leitura dos sensores



Ligacao do arduino:

Sensor:
Pino 2: S1
Pino 3: S2
Pino 4: S3
Pino 5: S4
Pino 6: S5

Pino 9: MOTOR DIREITO
Pino 10: MOTOR ESQUERDO



A partir da leitura do sensor infravermelho, foi estabelecido a seguinte 
logica de erro, onde o sensor 1 esta mais a esquerda e o 5 mais a direita:
0 0 0 0 1 ==> erro = 4
0 0 0 1 1 ==> erro = 3
0 0 0 1 0 ==> erro = 2
0 0 1 1 0 ==> erro = 1
0 0 1 0 0 ==> erro = 0  CONDICAO IDEAL
0 1 1 0 0 ==> erro = -1
0 1 0 0 0 ==> erro = -2
1 1 0 0 0 ==> erro = -3
1 0 0 0 0 ==> erro = -4



Controlador PID:
PID = (Kp*P)+(Ki*I)+(Kd*D), onde Kp, Ki e Kd sao completamente ajustaveis 
e foram encontradas por meio de tentativa e erro
*/

#include <avr/io.h> // biblioteca de I/O


// CALCULO DO ERRO A PARTIR DA LEITURA DOS SENSORES

int erro = 0;
int erroanterior = 0;

//Definindo as constantes de verificacao do erro de acordo com o pino do arduino
#define S1 PD2    // pino digital 2, sensor 1  0 - em cima da linha, 1 - fora da linha
#define S2 PD3    // pino digital 3, sensor 2
#define S3 PD4    // pino digital 4, sensor 3
#define S4 PD5    // pino digital 5, sensor 4
#define S5 PD6    // pino digital 6, sensor 5

int LS1, LS2, LS3, LS4, LS5; // varaiveis para verificar o valor logico de cada sensor
 // funcao para ler o e verificar o valor logico de cada sensor
void LeituraSensor(){
	if(PIND & (1<<S1)) LS1 = 1; //bitwise para verificar se S1 esta ou nao ativado
    else LS1 = 0;
	if(PIND & (1<<S2)) LS2 = 1; // verifica S2
    else LS2 = 0;
	if(PIND & (1<<S3)) LS3 = 1; // verifica S3
    else LS1 = 0;
	if(PIND & (1<<S4)) LS4 = 1; // verifica S4
    else LS4 = 0;
	if(PIND & (1<<S5)) LS5 = 1; // verifica S5
    else LS5 = 0;
}

// funcao para calcular o erro a partir das variaveis de leitura de ativacao do sensor
void calc_erro() {
	if(LS1 && LS2 && LS3 && LS4 && !LS5){  //caso 0 0 0 0 1
    erro = 4;
  }
  else if(LS1 && LS2 && LS3 && !LS4 && !LS5){ // caso 0 0 0 1 1 
    erro = 3;
  }
  else if(LS1 && LS2 && LS3 && !LS4 && LS5){ // caso 0 0 0 1 0
    erro = 2;
  }
  else if(LS1 && LS2 && !LS3 && !LS4 && LS5){ //caso 0 0 1 1 0 
    erro = 1;
  }
  else if(LS1 && LS2 && !LS3 && LS4 && LS5){ // caso 0 0 1 0 0 
    erro = 0;
  }
  else if(LS1 && !LS2 && !LS3 && LS4 && LS5){ // caso 0 1 1 0 0
    erro = -1;
  }
  else if(LS1 && !LS2 && LS3 && LS4 && LS5){ // caso 0 1 0 0 0 
    erro = -2;
  }
  else if(!LS1 && !LS2 && LS3 && LS4 && LS5){ // caso 1 1 0 0 0 
    erro = -3;
  }
  else if(!LS1 && LS2 && LS3 && LS4 && LS5){ // caso 1 0 0 0 0 
    erro = -4;
  }
}


// CALCULO DO CONTROLADOR PID A PARTIR DO ERRO

//Criancao das constantes do controaldor e iniciando-as em 0
double PID; // controlador
int I = 0; // integral
int P = 0; // proporcional 
int D = 0; // derivativo

// Valores das constantes de controle encontadas por tentativa e erro
// podendo variar de acordo com o projeto
#define Kp 70
#define	Ki 0.000055
#define	Kd 100


// Funcao para calcular o controlador PID, bem como suas variaveis
void controlador(){
	P = erro;
	I = I + erro;
	D = erro - erroanterior;
	erroanterior = erro;
	PID = (Kp*P)+(Ki*I)+(Kd*D);
}

// valor da velocidade media, tambem podendo variar de acordo com o projeto
int velocidade = 300;


// ATIVACAO DA FUNCAO DE SINAL DE SAIDA PWM


/* De acordo com a tabela 20-6. Waveform Generation Mode Bit Description da pagina 172 do manual do ATMega:
WGM11 e WGM10 indicam a ativacao do modo PWM de 10 bits

*/

//Funcao do sinal de saida do PWM
void iniciaPWM(){
	TCCR1A = ((1 << COM0A1) | (1 << COM0B1) | (1 << WGM11) | (1 << WGM10));
	TCCR1B = ((1 << CS11) | (1 << CS10)); 
}

// Funcao para escrever o sinal PWM
void escreveAnalogicoD9(int valorPWM){ // Escreve o valor PWM no pino 9
	OCR1A = valorPWM;     
}

// Funcao para escrever o sinal PWM
void escreveAnalogicoD10(int valorPWM){ // Escreve o valor PWM no pino 10
	OCR1B = valorPWM;    
}

int main (void){
	iniciaPWM(); // chama a funcao PWM
	DDRB = 0b11111111;  // seta todos os pinos da porta D como saida
	DDRD |= (0<<S1) | (0<<S2) | (0<<S3) | (0<<S4) | (0<<S5); //Seta os pinos 2 a 6 como entradas		

	while(1){ // looping infinito 
		LeituraSensor(); // chama a funcao de leitura do sensor
		calc_erro(); // chama a funcao de calculo do erro 
		controlador(); // chama a funcao do controlador 
		escreveAnalogicoD9(velocidade - (int)PID); //  emite um sinal pwm de acordo com o controlador no motor direito
		escreveAnalogicoD10(velocidade + (int)PID);//  emite um sinal pwm de acordo com o controlador no motor esquerdo
	}
}
