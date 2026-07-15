#include <stm32f4xx.h>
#include <stdio.h>
#include <string.h>

#define LCD_ADDR        0x27            /* Adresse I2C du module PCF8574 */
#define WIFI_SSID       "TOPNET_5ED0"
#define WIFI_PASS       "191919982761998"
#define TS_KEY          "7USUV7VSGNB7UTVK"
#define MQTT_BROKER     "192.168.1.15"
#define MQTT_PORT       "1883"
#define MQTT_CLIENT_ID  "STM32_Yassine"
#define IOT_PERIODE_S   30              

/* Resultats ADC remplis automatiquement par DMA (3 canaux : PC0, PC1, PC2) */
volatile uint16_t adc[3];

/* Drapeaux de synchronisation entre interruptions et boucle principale*/
volatile uint8_t  flag_adc_pret   = 0; /* 1 = nouvelles valeurs ADC disponibles*/
volatile uint8_t  flag_cmd_prete  = 0; /* 1 = commande Bluetooth recue complete */
volatile uint8_t  flag_iot_envoie = 0; /* 1 = periode IoT ecoulee*/
volatile uint32_t secondes        = 0; /* Compteur de secondes (incremente par TIM4) */

uint8_t iot_tour = 0; /* 0 = ThingSpeak, 1 = MQTT, alterne a chaque cycle */

/* Buffer de reception Bluetooth (commandes texte depuis HC-06) */
char     cmd_buf[50];
volatile int cmd_idx = 0;

/* Buffer d'emission Bluetooth via DMA(DMA lit en RAM) */
char     tx_buf[80];
volatile uint8_t tx_occupe = 0; /* Verrou DMA : 1 = transfert en cours */

/* Buffer de reponse ESP32 */
char esp_reponse[120];

/* Vitesse et registres PWM des 4 voies moteur */
uint16_t vitesse    = 200;
uint16_t CCR1, CCR2, CCR3, CCR4; /* Valeurs calculees avant mise dans TIM3 */

void Delay(int ncount);

void init_tim2_declencheur_adc(void);
void init_tim4_horloge_iot(void);
void init_exti0_bouton(void);
void init_adc1_dma(void);
void init_dma2_pour_adc(void);
void init_usart2_bluetooth(void);
void init_dma1_pour_uart2_tx(void);
void init_usart3_esp32(void);
void init_gpio_pwm(void);
void init_tim3_pwm(void);
void init_i2c2_lcd(void);
void init_priorites_nvic(void);

void delai_ms(uint32_t ms);
void uart2_envoie_dma(char *texte, uint16_t taille);
void esp32_envoie_et_lis(char *texte);

void wifi_connexion(void);
void iot_thingspeak(void);
void iot_mqtt(void);
void cmd_ROBOT(void);

void lcd_init(void);
void lcd_commande(char c);
void lcd_caractere(char c);
void lcd_texte(char *s);
void lcd_curseur(uint8_t ligne, uint8_t col);
void lcd_envoie_quartet(uint8_t data);

/* TIM2 : genere un evenement TRGO toutes les 3 secondes (16MHz/16000/3000)
   Il sert uniquement a declencher l'ADC  */
void init_tim2_declencheur_adc(void)
{
    RCC->APB1ENR |= (1 << 0);
    TIM2->PSC     = 15999; //f timer= fclock/psc+1 16MHZ/16
    TIM2->ARR     = 2999; //TIM2 compte 3000 ticks ... T=1s
    TIM2->CR2    |= TIM_CR2_MMS_1; /* Sortie TRGO sur evenement Update */
}
/* TIM4 : interruption chaque seconde (16MHz/16000/1000)
   Sert a compter les secondes et lever flag_iot_envoie toutes les 30s */
void init_tim4_horloge_iot(void)
{
    RCC->APB1ENR |= (1 << 2);
    TIM4->PSC     = 15999; // f timer= fclock/psc+1 16MHZ/16
    TIM4->ARR     = 999;
    TIM4->DIER   |= (1 << 0); /* Interruption sur debordement */
    TIM4->CR1    |= (1 << 0); /* Demarrer immediatement */
    NVIC_EnableIRQ(TIM4_IRQn);
}
/* EXTI0 : bouton sur PA0, front montant
   Demarre TIM2 pour lancer la premiere acquisition ADC */
void init_exti0_bouton(void)
{
    RCC->APB2ENR |= (1 << 14);/* Horloge SYSCFG */
    SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI0; /* EXTI0 sur GPIOA */
    EXTI->IMR  |= (1 << 0);/* Activer l'interruption */
    EXTI->RTSR |= (1 << 0);/* Front montant */
    NVIC_EnableIRQ(EXTI0_IRQn);
}
/* ADC1 : 3 canaux en scan (PC0=CH10, PC1=CH11, PC2=CH12)
   Mode continu avec DMA, declenche par TIM2 TRGO */
void init_adc1_dma(void)
{
    /* GPIO PC0, PC1, PC2 en mode analogique */
    RCC->AHB1ENR |= (1 << 2);
    GPIOC->MODER |= (3 << 0) | (3 << 2) | (3 << 4);
    RCC->APB2ENR |= (1 << 8); /* Horloge ADC1 */
    ADC1->CR1 = 0;
    ADC1->CR2 = 0;
    ADC1->CR1  |= (1 << 8);/* Mode SCAN : convertir tous les canaux de la sequence */
    ADC1->SQR1 |= (2 << 20);/* Sequence de 3 conversions (L = 2) */
    ADC1->SQR3  = (10 << 0) | (11 << 5) | (12 << 10); /* Ordre : CH10, CH11, CH12 */
    ADC1->SMPR1 |= (4 << 0) | (4 << 3) | (4 << 6);   /* Temps echantillonnage 84 cycles */
    ADC1->CR2 |= ADC_CR2_EXTEN_0;/* Declenchement sur front montant */
    ADC1->CR2 |= (6 << 24);/* Source declenchement : TIM2 TRGO */
    ADC1->CR2 |= (1 << 8);/* DMA mode */
    ADC1->CR2 |= (1 << 9);/* DMA requests continues */
    ADC1->CR2 |= ADC_CR2_ADON;/* Allumer l'ADC */
}

/* DMA2 Stream0 : transfert automatique ADC1->DR vers le tableau adc[]
   Circulaire, increment memoire, 16 bits, IRQ en fin de transfert */
void init_dma2_pour_adc(void)
{
    RCC->AHB1ENR |= (1 << 22);
    DMA2_Stream0->CR = 0;
    while (DMA2_Stream0->CR & 1); /* Attendre arret complet */
    DMA2_Stream0->CR |= (0 << 25); /* Channel 0 (ADC1) */
    DMA2_Stream0->CR |= (1 << 13); /* Taille memoire : 16 bits */
    DMA2_Stream0->CR |= (1 << 11); /* Taille peripherique : 16 bits */
    DMA2_Stream0->CR |= (1 << 10); /* Increment automatique de l'adresse memoire */
    DMA2_Stream0->CR |= (1 <<  8); /* Mode circulaire (recommence apres 3 transferts) */
    DMA2_Stream0->CR |= (0 <<  6); /* Direction : peripherique -> memoire */
    DMA2_Stream0->CR |= (1 <<  4); /* Interruption en fin de transfert complet */
    DMA2_Stream0->PAR  = (uint32_t)&ADC1->DR; /* Source : registre ADC */
    DMA2_Stream0->M0AR = (uint32_t)adc;/* Destination : tableau adc[3] */
    DMA2_Stream0->NDTR = 3;/* 3 transferts par cycle */
    DMA2_Stream0->FCR  = 0;/* Mode FIFO desactive */
    DMA2_Stream0->CR |= (1 << 0); /* Activer le DMA */
    NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

/* USART2 : liaison Bluetooth HC-06 sur PA2(TX) / PA3(RX)
   9600 bauds, emission par DMA, reception par interruption + detection IDLE */
void init_usart2_bluetooth(void)
{
    RCC->AHB1ENR |= (1 << 0); /* GPIOA */
    RCC->APB1ENR |= (1 << 17); /* USART2 */
    GPIOA->MODER &= ~((3 << 4) | (3 << 6));
    GPIOA->MODER |=  ((2 << 4) | (2 << 6)); /* PA2 et PA3 en mode AF */
    GPIOA->AFR[0] &= ~((0xF << 8) | (0xF << 12));
    GPIOA->AFR[0] |=  ((7 << 8) | (7 << 12)); /* AF7 = USART2 */
    USART2->BRR = 0x0683; /* 9600 bauds @ 16MHz */
    USART2->CR1 = 0;
    USART2->CR1 |= (1 << 3);  /* TE : activer l'emetteur */
    USART2->CR1 |= (1 << 2);  /* RE : activer le recepteur */
    USART2->CR1 |= (1 << 5);  /* RXNE IE : interruption reception */
    USART2->CR1 |= (1 << 4);  /* IDLE IE : interruption ligne inactive (fin de trame) */
    USART2->CR3 |= (1 << 7);  /* DMAT : emission via DMA */
    USART2->CR1 |= (1 << 13); /* UE : activer USART2 */
    NVIC_EnableIRQ(USART2_IRQn);
}
/* DMA1 Stream6 : transfert tx_buf[] vers USART2->DR
   Direction memoire->peripherique, sans circulaire (un coup par envoi) */
void init_dma1_pour_uart2_tx(void)
{
    RCC->AHB1ENR |= (1 << 21); /* DMA1 */
    DMA1_Stream6->CR = 0;//Efface ancienne configuration DMA.
    while (DMA1_Stream6->CR & 1);//Attendre désactivation comléte de dma
    DMA1_Stream6->CR |= (4 << 25); /* Channel 4 (USART2 TX) Relie DMA au USART2_TX. */
    DMA1_Stream6->CR |= (0 << 13); /* Taille memoire : 8 bits */
    DMA1_Stream6->CR |= (0 << 11); /* Taille peripherique : 8 bits */
    DMA1_Stream6->CR |= (1 << 10); /* Increment memoire */
    DMA1_Stream6->CR &= ~(1 << 8); /* Pas de mode circulaire */
    DMA1_Stream6->CR |= (1 <<  6); /* Direction : memoire -> peripherique */
    DMA1_Stream6->CR |= (1 <<  4); /* IRQ en fin de transfert */
    DMA1_Stream6->PAR = (uint32_t)&USART2->DR; //Destination DMA DR de usart2
    NVIC_EnableIRQ(DMA1_Stream6_IRQn);
}
/* USART3 : liaison avec l'ESP32 sur PC10(TX) / PC11(RX) */ 
 /* 115200 bauds, polling (pas de DMA ni d'IRQ) */
void init_usart3_esp32(void)
{
    RCC->APB1ENR |= (1 << 18); /* USART3 */
    RCC->AHB1ENR |= (1 << 2);  /* GPIOC */
    GPIOC->MODER &= ~((3 << 20) | (3 << 22));//On remet à zéro avant nouvelle configuration.
    GPIOC->MODER |=  ((2 << 20) | (2 << 22)); /* PC10 et PC11 en mode AF */
    GPIOC->AFR[1] &= ~((0xF << 8) | (0xF << 12));//Effacer ancienne fonction alternative
    GPIOC->AFR[1] |=  ((7 << 8) | (7 << 12)); /* Connecter pins à USART3 AF7 = USART3 */
    USART3->BRR = 0x8B; /* 115200 bauds @ 16MHz */
    USART3->CR1 = 0; //USART3->CR1 = 0
    USART3->CR1 |= (1 << 3);  /* TE Autoriser émission.*/
    USART3->CR1 |= (1 << 2);  /* RE Autoriser réception */
    USART3->CR1 |= (1 << 13); /* UE Démarrer USART3 */
}

/* GPIO pour PWM TIM3 : PA6(CH1), PA7(CH2), PB0(CH3), PB1(CH4) */
void init_gpio_pwm(void)
{
    RCC->AHB1ENR |= (1 << 0) | (1 << 1); /* Activer horloge GPIOA + GPIOB */
    GPIOA->MODER &= ~((3 << 12) | (3 << 14)); //Nettoyer configuration des pins.
    GPIOA->MODER |=  ((2 << 12) | (2 << 14)); //Relier pins au timer TIM3.
    GPIOA->AFR[0] &= ~((0xF << 24) | (0xF << 28));//Effacer ancienne fonction alternative.
    GPIOA->AFR[0] |=  ((2 << 24) | (2 << 28)); /* Associer pins à TIM3 AF2 = TIM3 */
    GPIOB->MODER &= ~((3 << 0) | (3 << 2)); //Nettoyer PB0 PB1.
    GPIOB->MODER |=  ((2 << 0) | (2 << 2)); //Relier PB0 PB1 au timer
    GPIOB->AFR[0] &= ~((0xF << 0) | (0xF << 4)); //Effacer ancienne fonction alternative
    GPIOB->AFR[0] |=  ((2 << 0) | (2 << 4));/* Associer pins à TIM3 AF2 = TIM3 */
}
/* TIM3 : 4 canaux PWM a 1 kHz (16MHz/16/1000)
   CCR1..CCR4 controle les 4 demi-ponts du pont en H */
void init_tim3_pwm(void)
{
    RCC->APB1ENR |= (1 << 1);
    TIM3->PSC     = 15; //  f/ PSC+1 = 16 16MHz / 16 = 1MHz
    TIM3->ARR     = 999; //Définir période PWM.
    /* Mode PWM 1 sur les 4 canaux (OC1M = 110) */
    TIM3->CCMR1  |= (6 << 4) | (6 << 12); // Configure :CH1 CH2 PWM en Mode 1
    TIM3->CCMR2  |= (6 << 4) | (6 << 12); // Configure :CH1 CH2 PWM en Mode 1
    TIM3->CCER   |= (1 << 0) | (1 << 4) | (1 << 8) | (1 << 12);/* Activer les 4 sorties */
    TIM3->CCR1 = TIM3->CCR2 = TIM3->CCR3 = TIM3->CCR4 = 0;/* Mettre PWM à 0% = Valeur initiale : tout a 0 (moteurs arretes) */
    TIM3->CR1 |= (1 << 0); /* Demarrer TIM3 */
}
/* I2C2 : bus pour le LCD 16x2 sur PB10(SCL) / PB11(SDA)
   100 kHz standard, mode open-drain avec pull-up interne */
void init_i2c2_lcd(void)
{
    RCC->AHB1ENR |= (1 << 1);  /* Activer GPIOB */
    RCC->APB1ENR |= (1 << 22); /* Activer I2C */
    GPIOB->MODER  &= ~((3 << 20) | (3 << 22));//Nettoyer MODER
    GPIOB->MODER  |=  ((2 << 20) | (2 << 22)); /* PB10/PB11 AF connecter pins au périphérique I2C2.*/
    GPIOB->OTYPER |=   (1 << 10) | (1 << 11);  /* Open-drain pour I2C */
    GPIOB->PUPDR  &= ~((3 << 20) | (3 << 22)); /* Désactiver pull internes (resistances externes) */
    GPIOB->OSPEEDR|=  (3 << 20) | (3 << 22);   /* Vitesse max Haute vitesse GPIO*/
    GPIOB->AFR[1] &= ~((0xF << 8) | (0xF << 12));// Nettoyer Alternate Function.
    GPIOB->AFR[1] |=  ((4 << 8) | (4 << 12));  /* AF4 = I2C2 ... Associer pins à I2C2. */
    I2C2->CR1  = I2C_CR1_SWRST; /* Réinitialiser bus I2C.*/
    I2C2->CR1  = 0; //Fin reset sortir de reset
    I2C2->CR2  = 16;   /* Frequence APB1 = 16 MHz */
    I2C2->CCR  = 80;   /* 1/16MHZ = 62.5ns* 80 = 5µs = Thigh = Tlow période total 10µs f=1/10us=100khz... Configurer vitesse I2C. */
    I2C2->TRISE = 17;  /* Temps de montee max : (1000ns max / 62.5ns) + 1 */
    I2C2->CR1 |= I2C_CR1_PE; /* Activer I2C2 */
}
/* Priorites NVIC : USART2 le plus urgent, DMA le moins urgent */
void init_priorites_nvic(void)
{NVIC_SetPriority(USART2_IRQn,0); //USART2 reçoit commandes robot
NVIC_SetPriority(EXTI0_IRQn,1);//Interruptions externes priorité élevée.
NVIC_SetPriority(TIM4_IRQn,2);//Timer périodique priorité moyenne horloge_iot
NVIC_SetPriority(DMA2_Stream0_IRQn,3); //IRQ DMA ADC priorité plus faible ADC moins critique que
NVIC_SetPriority(DMA1_Stream6_IRQn,4); //Fin transmission UART DMA priorité faible Car transmission TX non critique CPU peut attendre
}
/* Delai bloquant simple base sur une boucle vide ---- 16MHz=16 000 000 cycles/s ---1 cycle dure  1/16MHz= 62.5ns
1600 tours ˜ 1 ms
*/
void delai_ms(uint32_t ms)
{
for (volatile uint32_t i = 0; i < ms * 1600; i++);
}
/* Envoi non bloquant via DMA vers le Bluetooth HC-06.
   IMPORTANT : le pointeur passe doit etre global (tx_buf ou esp_reponse)
   car le DMA continue a lire la RAM apres le retour de cette fonction. */
void uart2_envoie_dma(char *texte, uint16_t taille)
{
    while (tx_occupe); /* Attendre que le precedent envoi soit -- termine Empêcher deux DMA simultanés. */
    tx_occupe = 1; //Verrouiller DMA -- signal qu'elle est occupé
    DMA1_Stream6->CR &= ~(1 << 0); //Stopper DMA avant reconfiguration.
    while (DMA1_Stream6->CR & 1); //Attendre arrêt complet car DMA prend quelques cycles pour s’arrêter physiquement.
    DMA1->HIFCR |= (0x3F << 16);/* Effacer les flags du stream 6 == High Interrupt Flag Clear Register  */
    DMA1_Stream6->M0AR = (uint32_t)texte; //Donner adresse mémoire source
    DMA1_Stream6->NDTR = taille; //Nombre de données à transférer
    DMA1_Stream6->CR  |= (1 << 0);/* Lancer le transfert */
    USART2->SR &= ~(1 << 6);/* Effacer flag Transmission Complete TC avant envoi */
}

/* Envoie une commande AT vers l'ESP32 a travers USART2 + DMA d'une facon non bloquante et attend sa reponse.
   - Timeout 3s pour le premier octet
   - Fin de reponse detectee par silence de 10ms
   - La reponse est ensuite relayee vers le Bluetooth pour monitoring */
void esp32_envoie_et_lis(char *texte)
{		int idx = 0; //Index de stockage c haque caractère reçu sera stocké dans esp_reponse[idx]
		uint32_t timeout;
/* Vider le buffer RX de tout octet residuel avant d'envoyer */
		while (USART3->SR & USART_SR_RXNE) //Lecture DR vide automatiquement RXNE
{volatile char flush = USART3->DR;
(void)flush;}
/* Envoyer la commande AT caractere par caractere */
	char *p = texte; //Pointeur parcours chaîne
	while (*p) //Tant que *p != '\0' continuer
{while ((USART3->SR & (1 << 7)) == 0);/* Attendre TXE TXE=0 USART encore occupé si non il est prêt a recevoir nouveau caractère. */
		USART3->DR = *p++;} //Envoi caractère
		timeout = 3000 * 1600;/* Attendre le premier octet de reponse (3 secondes max) */
while (!(USART3->SR & USART_SR_RXNE)) //tant que RXNE=0 attendre
{		
if (--timeout == 0) goto fin;} //Saut direct
/* Lire tous les octets jusqu'a 10ms de silence = fin de reponse */
while (idx < 118)
{
	timeout = 10 * 1600; //* Si 10ms sans nouveau caractèrealors réponse terminée */
while (!(USART3->SR & USART_SR_RXNE))
{
if (--timeout == 0) goto fin;
} 
esp_reponse[idx++] = (char)USART3->DR; //Lecture caractère
}
fin:
esp_reponse[idx] = '\0';
/* Relayer la reponse vers le telephone via Bluetooth */
if (idx > 0)
{while (tx_occupe); //Attendre tant que dma ocuppee
uart2_envoie_dma(esp_reponse, idx);
while (tx_occupe);}}

/* Connexion au reseau WiFi via commandes AT standard */
void wifi_connexion(void)
{
    delai_ms(2000);
    esp32_envoie_et_lis("AT\r\n");
    delai_ms(500);
    esp32_envoie_et_lis("AT+CWMODE=1\r\n"); /* Mode station (client WiFi) */
    delai_ms(500);
    esp32_envoie_et_lis("AT+CWJAP=\"" WIFI_SSID "\",\"" WIFI_PASS "\"\r\n");
    delai_ms(10000); /* Laisser le temps de s'associer */
    esp32_envoie_et_lis("AT+CIFSR\r\n");    /* Verifier qu'une IP a ete obtenue */
    delai_ms(500);
}

/* Envoi des 3 valeurs ADC vers ThingSpeak via HTTP GET.
   La requete doit avoir Host + Connection: close + ligne vide finale. */
void iot_thingspeak(void)
{
    char requete[200];
    char cmd[50];
    sprintf(requete,
        "GET /update?api_key=" TS_KEY
        "&field1=%u&field2=%u&field3=%u HTTP/1.1\r\n"
        "Host: api.thingspeak.com\r\n"
        "Connection: close\r\n"
        "\r\n",
        (unsigned)adc[0], (unsigned)adc[1], (unsigned)adc[2]);
    esp32_envoie_et_lis("AT+CIPSTART=\"TCP\",\"api.thingspeak.com\",80\r\n");
    delai_ms(2000);
    sprintf(cmd, "AT+CIPSEND=%d\r\n", (int)strlen(requete));
    esp32_envoie_et_lis(cmd);
    delai_ms(1500); /* Attendre le prompt ">" avant d'envoyer les donnees */
    esp32_envoie_et_lis(requete);
    delai_ms(4000);
    esp32_envoie_et_lis("AT+CIPCLOSE\r\n");
    delai_ms(500);
}

/* Envoi des 3 valeurs ADC vers un broker MQTT local (Mosquitto).
   scheme=1 dans MQTTUSERCFG = MQTT sans TLS.
   Topics : robot/adc_1, robot/adc_2, robot/adc_3 */
void iot_mqtt(void)
{
    char cmd[130];
    char val[16];

    /* Configurer le client MQTT sans authentification */
    esp32_envoie_et_lis(
        "AT+MQTTUSERCFG=0,1,\"" MQTT_CLIENT_ID "\",\"\",\"\",0,0,\"\"\r\n");
    delai_ms(1000);

    /* Se connecter au broker */
    sprintf(cmd, "AT+MQTTCONN=0,\"" MQTT_BROKER "\"," MQTT_PORT ",0\r\n");
    esp32_envoie_et_lis(cmd);
    delai_ms(5000); /* Le broker local peut prendre un peu de temps */
    /* Publier les 3 valeurs ADC sur des topics distincts */
    sprintf(val, "%u", (unsigned)adc[0]);
    sprintf(cmd, "AT+MQTTPUB=0,\"robot/adc_1\",\"%s\",1,0\r\n", val);
    esp32_envoie_et_lis(cmd);
    delai_ms(800);
    sprintf(val, "%u", (unsigned)adc[1]);
    sprintf(cmd, "AT+MQTTPUB=0,\"robot/adc_2\",\"%s\",1,0\r\n", val);
    esp32_envoie_et_lis(cmd);
    delai_ms(800);

    sprintf(val, "%u", (unsigned)adc[2]);
    sprintf(cmd, "AT+MQTTPUB=0,\"robot/adc_3\",\"%s\",1,0\r\n", val);
    esp32_envoie_et_lis(cmd);
    delai_ms(800);

    /* Fermer proprement la session MQTT */
    esp32_envoie_et_lis("AT+MQTTCLEAN=0\r\n");
    delai_ms(500);
}
/* Interprete la commande recue via Bluetooth et pilote les moteurs.
   Vitesse progressive : +100 a chaque commande de deplacement.
   Moteur droit  = CCR1 (avant) / CCR2 (arriere)
   Moteur gauche = CCR3 (avant) / CCR4 (arriere) */
void cmd_ROBOT(void)
{
    /* Augmentation progressive de la vitesse a chaque deplacement */
    if (strstr(cmd_buf, "AVANCE")  ||
        strstr(cmd_buf, "ARRIERE") ||
        strstr(cmd_buf, "GAUCHE")  ||
        strstr(cmd_buf, "DROITE"))
    {
        if (vitesse < 900) vitesse += 100;
        else vitesse = 900;
    }
    if (strstr(cmd_buf, "AVANCE"))
    {
        CCR1 = vitesse; CCR2 = 0;   /* Moteur droit avant */
        CCR3 = vitesse; CCR4 = 0;   /* Moteur gauche avant */
    }
    else if (strstr(cmd_buf, "ARRIERE"))
    {
        CCR1 = 0; CCR2 = vitesse;   /* Moteur droit arriere */
        CCR3 = 0; CCR4 = vitesse;   /* Moteur gauche arriere */
    }
    /*  GAUCHE : moteur droit plus rapide = virage gauche  */
    else if (strstr(cmd_buf, "GAUCHE"))
    {
        CCR1 = vitesse;     CCR2 = 0; /* Droit a pleine vitesse */
        CCR3 = vitesse / 2; CCR4 = 0; /* Gauche ralenti */
    }
    /*  DROITE : moteur gauche plus rapide = virage droit  */
    else if (strstr(cmd_buf, "DROITE"))
    {
        CCR1 = vitesse / 2; CCR2 = 0; /* Droit ralenti */
        CCR3 = vitesse;     CCR4 = 0; /* Gauche a pleine vitesse */
    }
    /*  STOP : tout couper, reinitialiser la vitesse  */
    else if (strstr(cmd_buf, "STOP"))
    {
        vitesse = 200;
        CCR1 = 0; CCR2 = 0;
        CCR3 = 0; CCR4 = 0;
    }
    /* Mise a jour des registres PWM */
    TIM3->CCR1 = CCR1;
    TIM3->CCR2 = CCR2;
    TIM3->CCR3 = CCR3;
    TIM3->CCR4 = CCR4;
    /* Confirmer l'execution de la commande au telephone */
    sprintf(tx_buf, "CMD:%s|V=%u\r\n", cmd_buf, vitesse);
    uart2_envoie_dma(tx_buf, strlen(tx_buf));
    cmd_buf[0] = '\0'; /* Effacer le buffer pour la prochaine commande */
}
/* Envoie un quartet (4 bits + bits de controle) sur le bus I2C */
void lcd_envoie_quartet(uint8_t data)
{
    while (!(I2C2->SR1 & I2C_SR1_TXE));
    I2C2->DR = data;
    for (volatile int i = 0; i < 200; i++); /* Petit delai pour le pulse EN */
    while (!(I2C2->SR1 & I2C_SR1_TXE));
}
/* Envoie un octet de commande au LCD (RS = 0) */
void lcd_commande(char c)
{
    char haut = c & 0xF0;
    char bas  = (c << 4) & 0xF0;
    I2C2->CR1 |= I2C_CR1_START;
    while (!(I2C2->SR1 & I2C_SR1_SB));
    I2C2->DR = LCD_ADDR << 1;
    while (!(I2C2->SR1 & I2C_SR1_ADDR));
    volatile int tmp = I2C2->SR2; (void)tmp;
    lcd_envoie_quartet(haut | 0x0C); /* EN=1, RS=0, BL=1 */
    lcd_envoie_quartet(haut | 0x08); /* EN=0 */
    lcd_envoie_quartet(bas  | 0x0C);
    lcd_envoie_quartet(bas  | 0x08);
    I2C2->CR1 |= I2C_CR1_STOP;
}

/* Envoie un caractere affichable au LCD (RS = 1) */
void lcd_caractere(char c)
{
    char haut = c & 0xF0;
    char bas  = (c << 4) & 0xF0;

    I2C2->CR1 |= I2C_CR1_START;
    while (!(I2C2->SR1 & I2C_SR1_SB));
    I2C2->DR = LCD_ADDR << 1;
    while (!(I2C2->SR1 & I2C_SR1_ADDR));
    volatile int tmp = I2C2->SR2; (void)tmp;

    lcd_envoie_quartet(haut | 0x0D); /* EN=1, RS=1, BL=1 */
    lcd_envoie_quartet(haut | 0x09); /* EN=0 */
    lcd_envoie_quartet(bas  | 0x0D);
    lcd_envoie_quartet(bas  | 0x09);

    I2C2->CR1 |= I2C_CR1_STOP;
}

/* Affiche une chaine de caracteres a la position courante */
void lcd_texte(char *s)
{
    while (*s) lcd_caractere(*s++);
}

/* Positionne le curseur sur la ligne (0 ou 1) et la colonne voulue */
void lcd_curseur(uint8_t ligne, uint8_t col)
{
    uint8_t pos = (ligne == 0) ? (0x80 + col) : (0xC0 + col);
    lcd_commande(pos);
}

/* Sequence d'initialisation obligatoire pour le HD44780 en mode 4 bits */
void lcd_init(void)
{
    delai_ms(50);                   /* Attendre la mise sous tension */
    lcd_commande(0x30); delai_ms(5); /* Reset #1 */
    lcd_commande(0x30); delai_ms(1); /* Reset #2 */
    lcd_commande(0x30); delai_ms(1); /* Reset #3 */
    lcd_commande(0x20); delai_ms(1); /* Passer en mode 4 bits */
    lcd_commande(0x28);              /* 4 bits, 2 lignes, police 5x8 */
    lcd_commande(0x08);              /* Ecran eteint (pour effacer) */
    lcd_commande(0x01); delai_ms(2); /* Effacer l'ecran */
    lcd_commande(0x06);              /* Curseur avance vers la droite */
    lcd_commande(0x0C);              /* Ecran allume, curseur cache */
}

/* ================================================================
   MAIN
   ================================================================ */

int main(void)
{
    /* --- Initialisation de tous les peripheriques --- */
    init_exti0_bouton();
    init_tim2_declencheur_adc();
    init_adc1_dma();
    init_dma2_pour_adc();
    init_usart2_bluetooth();
    init_dma1_pour_uart2_tx();
    init_usart3_esp32();
    init_gpio_pwm();
    init_tim3_pwm();
    init_i2c2_lcd();
    init_tim4_horloge_iot();
    init_priorites_nvic();

    /* --- Ecran d'accueil et connexion WiFi --- */
    lcd_init();
    delai_ms(300);
    lcd_curseur(0, 0); lcd_texte("Robot IoT  v4.0");
    lcd_curseur(1, 0); lcd_texte("WiFi connect...");

    wifi_connexion();

    lcd_curseur(1, 0); lcd_texte("WiFi OK!       ");
    delai_ms(1000);

    char ligne_lcd[17];

    /* ============================================================
       BOUCLE PRINCIPALE
       Traite les drapeaux leves par les interruptions.
       Ordre de priorite : commandes robot > ADC > IoT
       ============================================================ */
    while (1)
    {
        /* 1. Commande Bluetooth recue ? L'executer immediatement */
        if (flag_cmd_prete)
        {
            flag_cmd_prete = 0;
            cmd_ROBOT();
        }

        /* 2. Nouvelles valeurs ADC ? Mettre a jour le LCD et le telephone */
        if (flag_adc_pret)
        {
            flag_adc_pret = 0;

            lcd_curseur(1, 0);
            sprintf(ligne_lcd, "%4u %4u %4u  ",
                    (unsigned)adc[0], (unsigned)adc[1], (unsigned)adc[2]);
            lcd_texte(ligne_lcd);

            sprintf(tx_buf, "ADC:%u %u %u\r\n",
                    (unsigned)adc[0], (unsigned)adc[1], (unsigned)adc[2]);
            uart2_envoie_dma(tx_buf, strlen(tx_buf));
        }

        /* 3. Periode IoT ecoulee ? Alterner ThingSpeak et MQTT */
        if (flag_iot_envoie)
        {
            flag_iot_envoie = 0;

            if (iot_tour == 0)
            {
                lcd_curseur(0, 0); lcd_texte("->ThingSpeak... ");
                iot_thingspeak();

                sprintf(tx_buf, "TS OK:%u %u %u\r\n",
                        (unsigned)adc[0], (unsigned)adc[1], (unsigned)adc[2]);
                uart2_envoie_dma(tx_buf, strlen(tx_buf));
                iot_tour = 1;
            }
            else
            {
                lcd_curseur(0, 0); lcd_texte("->MQTT local... ");
                iot_mqtt();

                sprintf(tx_buf, "MQTT OK:%u %u %u\r\n",
                        (unsigned)adc[0], (unsigned)adc[1], (unsigned)adc[2]);
                uart2_envoie_dma(tx_buf, strlen(tx_buf));
                iot_tour = 0;
            }

            lcd_curseur(0, 0); lcd_texte("Robot IoT  OK   ");
        }
    }
}

/* ================================================================
   GESTIONNAIRES D'INTERRUPTION
   ================================================================ */

/* Bouton PA0 : demarre TIM2 pour declencher la premiere conversion ADC */
void EXTI0_IRQHandler(void)
{
    if (EXTI->PR & (1 << 0))
    {
        TIM2->CR1 |= (1 << 0); /* Demarrer TIM2 */
        EXTI->PR   = (1 << 0); /* Effacer le pending */
    }
}

/* TIM4 (chaque seconde) : incremente le compteur IoT */
void TIM4_IRQHandler(void)
{
    if (TIM4->SR & (1 << 0))
    {
        TIM4->SR &= ~(1 << 0);
        secondes++;
        if (secondes >= IOT_PERIODE_S)
        {
            secondes = 0;
            flag_iot_envoie = 1; /* Signal pour la boucle principale */
        }
    }
}

/* DMA2 Stream0 : fin de transfert ADC -> adc[] (3 canaux acquis) */
void DMA2_Stream0_IRQHandler(void)
{
    if (DMA2->LISR & (1 << 5))
    {
        DMA2->LIFCR  |= (1 << 5);
        flag_adc_pret = 1;
    }
}

/* DMA1 Stream6 : fin de transfert Bluetooth, liberer le verrou */
void DMA1_Stream6_IRQHandler(void)
{
    if (DMA1->HISR & (1 << 21))
    {
        DMA1->HIFCR |= (1 << 21);
        tx_occupe = 0; /* Envoi termine, buffer disponible */
    }
}

/* USART2 : reception Bluetooth caractere par caractere.
   RXNE = nouveau caractere recu, IDLE = fin de trame (silence sur la ligne) */
void USART2_IRQHandler(void)
{
    /* Lire et stocker le caractere recu */
    if (USART2->SR & USART_SR_RXNE)
    {
        char c = USART2->DR;
        if (cmd_idx < (int)(sizeof(cmd_buf) - 1))
        {
            cmd_buf[cmd_idx++] = c;
            cmd_buf[cmd_idx]   = '\0';
        }
        else
        {
            cmd_idx = 0; /* Buffer plein : recommencer */
        }
    }

    /* Ligne inactive = fin de la commande, signaler a la boucle principale */
    if (USART2->SR & USART_SR_IDLE)
    {
        volatile uint32_t tmp = USART2->SR; /* Lecture SR puis DR pour effacer IDLE */
        tmp = USART2->DR;
        (void)tmp;
        flag_cmd_prete = 1;
        cmd_idx = 0;
    }
}

/* Delai generique (utilise dans certains contextes anciens du projet) */
void Delay(int ncount)
{
    volatile int i, j;
    for (i = 0; i < ncount; i++)
        for (j = 0; j < 1000; j++);
}