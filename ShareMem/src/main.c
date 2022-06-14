/** @main main.c
 * @brief This program implements cooperative tasks in Zephyr. 
 *
 * 
 * It does a basic processing of an analog signal using shared
 * memory + Smaphores.
 *
 * @author Bruno Feitais
 * @date 2022/05
 * @bug There are no bugs
 */

#include <zephyr.h>
#include <device.h>
#include <drivers/gpio.h>
#include <drivers/pwm.h>
#include <sys/printk.h>
#include <sys/__assert.h>
#include <string.h>
#include <timing/timing.h>
#include <stdlib.h>
#include <stdio.h>
#include <devicetree.h>
#include <drivers/adc.h>

/** ADC definitions and includes */
#include <hal/nrf_saadc.h>
/** ADC definitions and includes */
#define ADC_NID DT_NODELABEL(adc)
/** ADC definitions and includes */ 
#define ADC_RESOLUTION 10
/** ADC definitions and includes */
#define ADC_GAIN ADC_GAIN_1_4
/** ADC definitions and includes */
#define ADC_REFERENCE ADC_REF_VDD_1_4
/** ADC definitions and includes */
#define ADC_ACQUISITION_TIME ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 40)
/** ADC definitions and includes */
#define ADC_CHANNEL_ID 1  

/* This is the actual nRF ANx input to use. Note that a channel can be assigned to any ANx. In fact a channel can */
/*    be assigned to two ANx, when differential reading is set (one ANx for the positive signal and the other one for the negative signal) */  
/* Note also that the configuration of differnt channels is completely independent (gain, resolution, ref voltage, ...) */
/** This is the actual nRF ANx input to use. Note that a channel can be assigned to any ANx.*/
#define ADC_CHANNEL_INPUT NRF_SAADC_INPUT_AIN1

/** Buffer size definition */
#define BUFFER_SIZE 1

/* Other defines */
/** Interval between ADC samples */
#define TIMER_INTERVAL_MSEC 1 /* Interval between ADC samples */

/**
 * Pin at which BUT1 is connected. Addressing is direct (i.e., pin number)
 */
#define BOARDBUT1 0xb  // Pin at which BUT1 is connected. Addressing is direct (i.e., pin number) 
/**
 * Pin at which BUT2 is connected. Addressing is direct (i.e., pin number)
 */
#define BOARDBUT2 0xc  // Pin at which BUT2 is connected. Addressing is direct (i.e., pin number) 
/**
 * Pin at which BUT3 is connected. Addressing is direct (i.e., pin number)
 */
#define BOARDBUT3 0x18 // Pin at which BUT3 is connected. Addressing is direct (i.e., pin number) 
/**
 * Pin at which BUT4 is connected. Addressing is direct (i.e., pin number)
 */
#define BOARDBUT4 0x19 // Pin at which BUT4 is connected. Addressing is direct (i.e., pin number) 

/** ADC channel configuration */
static const struct adc_channel_cfg my_channel_cfg = {
	.gain = ADC_GAIN,
	.reference = ADC_REFERENCE,
	.acquisition_time = ADC_ACQUISITION_TIME,
	.channel_id = ADC_CHANNEL_ID,
	.input_positive = ADC_CHANNEL_INPUT
};

// Int related declarations 
static struct gpio_callback but1_cb_data; // Callback structure 
static struct gpio_callback but2_cb_data; // Callback structure 
static struct gpio_callback but3_cb_data; // Callback structure 
static struct gpio_callback but4_cb_data; // Callback structure 


/**
 * Flag to signal a BUT1 press
 */
volatile int botao1 = 0;  // Flag to signal a BUT1 press 
/**
 * Flag to signal a BUT2 press
 */
volatile int botao2 = 0; // Flag to signal a BUT2 press 
/**
 * Flag to signal a BUT3 press
 */
volatile int botao3 = 0; // Flag to signal a BUT3 press
/**
 * Flag to signal a BUT4 press
 */ 
volatile int botao4 = 0; // Flag to signal a BUT4 press

volatile int flag1 = 0;
volatile int flag2 = 0;

/** Refer to dts file */
#define GPIO0_NID DT_NODELABEL(gpio0)
/** Refer to dts file */
#define PWM0_NID DT_NODELABEL(pwm0)
/** Refer to dts file */
#define BOARDLED1 0xd /* Pin at which LED1 is connected.  Addressing is direct (i.e., pin number) */

/** Size of stack area used by each thread (can be thread specific)*/
#define STACK_SIZE 1024

/** Thread scheduling priority */
#define thread_A_prio 1
/** Thread scheduling priority */
#define thread_B_prio 1
/** Thread scheduling priority */
#define thread_C_prio 1
/** Thread scheduling priority */
#define thread_D_prio 1
/** Thread scheduling priority */
#define thread_E_prio 1
/** Thread scheduling priority */
#define thread_F_prio 1

/** Therad periodicity (in ms)*/
#define thread_A_period 100
/** Therad periodicity (in ms)*/
#define thread_D_period 1000

/* Global vars */
struct k_timer my_timer;
const struct device *adc_dev = NULL;
static uint16_t adc_sample_buffer[BUFFER_SIZE];

/** Create thread stack space */
K_THREAD_STACK_DEFINE(thread_A_stack, STACK_SIZE);
/** Create thread stack space */
K_THREAD_STACK_DEFINE(thread_B_stack, STACK_SIZE);
/** Create thread stack space */
K_THREAD_STACK_DEFINE(thread_C_stack, STACK_SIZE);
/** Create thread stack space */
K_THREAD_STACK_DEFINE(thread_D_stack, STACK_SIZE);
/** Create thread stack space */
K_THREAD_STACK_DEFINE(thread_E_stack, STACK_SIZE);
/** Create thread stack space */
K_THREAD_STACK_DEFINE(thread_F_stack, STACK_SIZE);

/* Create variables for thread data */
struct k_thread thread_A_data;
struct k_thread thread_B_data;
struct k_thread thread_C_data;
struct k_thread thread_D_data;
struct k_thread thread_E_data;
struct k_thread thread_F_data;


/* Create task IDs */
k_tid_t thread_A_tid;
k_tid_t thread_B_tid;
k_tid_t thread_C_tid;
k_tid_t thread_D_tid;
k_tid_t thread_E_tid;
k_tid_t thread_F_tid;


/* Global vars (shared memory between tasks A/B and B/C, resp) */
int DadosDE[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int DadosBC = 0;
int DadosBC2 = 100;
int DadosEF = 0;

/* Semaphores for task synch */
struct k_sem sem_ab;
struct k_sem sem_bc;
struct k_sem sem_ad;
struct k_sem sem_de;
struct k_sem sem_ef;


/** Takes one sample */
static int adc_sample(void)
{
	int ret;
	const struct adc_sequence sequence = {
		.channels = BIT(ADC_CHANNEL_ID),
		.buffer = adc_sample_buffer,
		.buffer_size = sizeof(adc_sample_buffer),
		.resolution = ADC_RESOLUTION,
	};

	if (adc_dev == NULL) {
            printk("adc_sample(): error, must bind to adc first \n\r");
            return -1;
	}

	ret = adc_read(adc_dev, &sequence);
	if (ret) {
            printk("adc_read() failed with code %d\n", ret);
	}	

	return ret;
}

void but1press_cbfunction(const struct device *dev, struct gpio_callback *cb, uint32_t pins){
    
    // Inform that button was hit
    printk("But1 pressed at %d\n\r", k_cycle_get_32());
    
    // Update Flag
    botao1 = 1;
}
/**
 * Function butxpress_cbfunction
 *
 * This function detects if the button was pressed and it prints in the terminal if the button was pressed.
 * It also has a flag that becomes 1 if the button is pressed.
 */
void but2press_cbfunction(const struct device *dev, struct gpio_callback *cb, uint32_t pins){
    
    // Inform that button was hit
    printk("But2 pressed at %d\n\r", k_cycle_get_32());
    
    // Update Flag
    botao2 = 1;
}
/**
 * Function butxpress_cbfunction
 *
 * This function detects if the button was pressed and it prints in the terminal if the button was pressed.
 * It also has a flag that becomes 1 if the button is pressed.
 */
void but3press_cbfunction(const struct device *dev, struct gpio_callback *cb, uint32_t pins){
    
    // Inform that button was hit
    printk("But3 pressed at %d\n\r", k_cycle_get_32());
    
    // Update Flag
    botao3 = 1;
}
/**
 * Function butxpress_cbfunction
 *
 * This function detects if the button was pressed and it prints in the terminal if the button was pressed.
 * It also has a flag that becomes 1 if the button is pressed.
 */
void but4press_cbfunction(const struct device *dev, struct gpio_callback *cb, uint32_t pins){
    
    // Inform that button was hit
    printk("But4 pressed at %d\n\r", k_cycle_get_32());
    
    // Update Flag
    botao4 = 1;
}

void ConfigurePins() {
    // Local vars
    const struct device *gpio0_dev;     // Pointer to GPIO device structure 
    int ret=0;                          // Generic return value variable  

    // Bind to GPIO 0 and PWM0
    gpio0_dev = device_get_binding(DT_LABEL(GPIO0_NID));
    if (gpio0_dev == NULL) {
        printk("Error: Failed to bind to GPIO0\n\r");        
	return;
    }
    else {
        printk("Bind to GPIO0 successfull \n\r");        
    }

    // Configure PINS
    ret = gpio_pin_configure(gpio0_dev, BOARDBUT1, GPIO_INPUT | GPIO_PULL_UP);
    if (ret < 0) {
        printk("Error %d: Failed to configure BUT 1 \n\r", ret);
	return;
    }
    ret = gpio_pin_configure(gpio0_dev, BOARDBUT2, GPIO_INPUT | GPIO_PULL_UP);
    if (ret < 0) {
        printk("Error %d: Failed to configure BUT 2 \n\r", ret);
	return;
    }
    ret = gpio_pin_configure(gpio0_dev, BOARDBUT3, GPIO_INPUT | GPIO_PULL_UP);
    if (ret < 0) {
        printk("Error %d: Failed to configure BUT 3 \n\r", ret);
	return;
    }
    ret = gpio_pin_configure(gpio0_dev, BOARDBUT4, GPIO_INPUT | GPIO_PULL_UP);
    if (ret < 0) {
        printk("Error %d: Failed to configure BUT 4 \n\r", ret);
	return;
    }

    // Set interrupt HW - which pin and event generate interrupt 
    ret = gpio_pin_interrupt_configure(gpio0_dev, BOARDBUT1, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret != 0) {
	printk("Error %d: failed to configure interrupt on BUT1 pin \n\r", ret);
	return;
    }
    ret = gpio_pin_interrupt_configure(gpio0_dev, BOARDBUT2, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret != 0) {
	printk("Error %d: failed to configure interrupt on BUT2 pin \n\r", ret);
	return;
    }
    ret = gpio_pin_interrupt_configure(gpio0_dev, BOARDBUT3, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret != 0) {
	printk("Error %d: failed to configure interrupt on BUT3 pin \n\r", ret);
	return;
    }
    ret = gpio_pin_interrupt_configure(gpio0_dev, BOARDBUT4, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret != 0) {
	printk("Error %d: failed to configure interrupt on BUT4 pin \n\r", ret);
	return;
    }

    
    // Set callback 
    gpio_init_callback(&but1_cb_data, but1press_cbfunction, BIT(BOARDBUT1));
    gpio_init_callback(&but2_cb_data, but2press_cbfunction, BIT(BOARDBUT2));
    gpio_init_callback(&but3_cb_data, but3press_cbfunction, BIT(BOARDBUT3));
    gpio_init_callback(&but4_cb_data, but4press_cbfunction, BIT(BOARDBUT4));
    gpio_add_callback(gpio0_dev, &but1_cb_data);
    gpio_add_callback(gpio0_dev, &but2_cb_data);
    gpio_add_callback(gpio0_dev, &but3_cb_data);
    gpio_add_callback(gpio0_dev, &but4_cb_data);

    return;
}

/* Thread code prototypes */
void thread_A_code(void *argA, void *argB, void *argC);
void thread_B_code(void *argA, void *argB, void *argC);
void thread_C_code(void *argA, void *argB, void *argC);
void thread_D_code(void *argA, void *argB, void *argC);
void thread_E_code(void *argA, void *argB, void *argC);
void thread_F_code(void *argA, void *argB, void *argC);


/** Main function */
void main(void) {

    int err = 0;

    /* ADC setup: bind and initialize */
    adc_dev = device_get_binding(DT_LABEL(ADC_NID));
	if (!adc_dev) {
        printk("ADC device_get_binding() failed\n");
    } 
    err = adc_channel_setup(adc_dev, &my_channel_cfg);
    if (err) {
        printk("adc_channel_setup() failed with error code %d\n", err);
    }

    /* Welcome message */
    printf("\n\r Illustration of the use of shmem + semaphores\n\r");
    
    /* Create and init semaphores */
    k_sem_init(&sem_ab, 0, 1);
    k_sem_init(&sem_bc, 0, 1);
    k_sem_init(&sem_ad, 0, 1);
    k_sem_init(&sem_de, 0, 1);
    k_sem_init(&sem_ef, 0, 1);
    
    /* Create tasks */
    thread_A_tid = k_thread_create(&thread_A_data, thread_A_stack,
        K_THREAD_STACK_SIZEOF(thread_A_stack), thread_A_code,
        NULL, NULL, NULL, thread_A_prio, 0, K_NO_WAIT);

    thread_B_tid = k_thread_create(&thread_B_data, thread_B_stack,
        K_THREAD_STACK_SIZEOF(thread_B_stack), thread_B_code,
        NULL, NULL, NULL, thread_B_prio, 0, K_NO_WAIT);

    thread_C_tid = k_thread_create(&thread_C_data, thread_C_stack,
        K_THREAD_STACK_SIZEOF(thread_C_stack), thread_C_code,
        NULL, NULL, NULL, thread_C_prio, 0, K_NO_WAIT);

    thread_D_tid = k_thread_create(&thread_D_data, thread_D_stack,
        K_THREAD_STACK_SIZEOF(thread_D_stack), thread_D_code,
        NULL, NULL, NULL, thread_D_prio, 0, K_NO_WAIT);
    
    thread_E_tid = k_thread_create(&thread_E_data, thread_E_stack,
        K_THREAD_STACK_SIZEOF(thread_E_stack), thread_E_code,
        NULL, NULL, NULL, thread_E_prio, 0, K_NO_WAIT);

    thread_F_tid = k_thread_create(&thread_F_data, thread_F_stack,
        K_THREAD_STACK_SIZEOF(thread_F_stack), thread_F_code,
        NULL, NULL, NULL, thread_F_prio, 0, K_NO_WAIT);

    return;
} 

/** Thread A code implementation. 
 * It reads 10 ADC values and saves it on the shared memory. */
void thread_A_code(void *argA , void *argB, void *argC)
{
    /* Timing variables to control task periodicity */
    int64_t fin_time=0, release_time=0;

    release_time = k_uptime_get() + thread_A_period;

    /* Other variables */
    long int nact = 0;
    int err = 0;

    /* Thread loop */
    while(1) {
        printk("\n\nLeitura Botoes (Thread A)\n");

        ConfigurePins();

        if(botao1 == 1){
          botao1 = 0;
          flag1 = 1;
          flag2 = 0;
        }
        if(botao2 == 1){
          botao2 = 0;
          flag1 = 0;
          flag2 = 1;
        }
        
        if(flag1 == 1){
          printk("Manual %d %d\n", flag1, flag2);
          k_sem_give(&sem_ab);
        }
        if(flag2 == 1){
          printk("Automatico %d %d\n", flag1, flag2);
          k_sem_give(&sem_ad);
        }

        botao1 = 0;
        botao2 = 0;

        /* Wait for next release instant */ 
        fin_time = k_uptime_get();
        if( fin_time < release_time) {
            k_msleep(release_time - fin_time);
            release_time += thread_A_period;
        }
    }
}

/** Thread B code implementation. 
 * It gets the 10 ADC values, does the average and saves it on the shared memory. */
void thread_B_code(void *argA , void *argB, void *argC)
{
    /* Other variables */
    long int nact = 0;

    while(1) {
        k_sem_take(&sem_ab,  K_FOREVER);
        printk("Atribuir intensidade luz (Thread B)\n");

        ConfigurePins();


        /* printk("\n Botao1 = %d", botao1);
        printk(" | Botao2 = %d", botao2);
        printk(" | Botao3 = %d", botao3);
        printk(" | Botao4 = %d\n", botao4); */
        
        if(botao3 == 1 && DadosBC >= 5){
          botao3 = 0;
          DadosBC = DadosBC - 5;
          DadosBC2 = DadosBC2 + 5;
        }

        if(botao4 == 1 && DadosBC <= 95){
          botao4 = 0;
          DadosBC += 5;
          DadosBC2 = DadosBC2 - 5;
        }
        
        botao3 = 0;
        botao4 = 0;
        k_sem_give(&sem_bc);
    }
}

/** Thread C code implementation. 
 * It reads the average and implements it on the LED 1. */
void thread_C_code(void *argA , void *argB, void *argC)
{
    /* Other variables */
    const struct device *pwm0_dev;          /* Pointer to PWM device structure */
    int pwm0_channel  = 13;                 /* Ouput pin associated to pwm channel. See DTS for pwm channel - output pin association */
     
    unsigned int pwmPeriod_us = 1000;       /* PWM period in us */
    int ret = 0;
    long int nact = 0;

    pwm0_dev = device_get_binding(DT_LABEL(PWM0_NID));
    if (pwm0_dev == NULL) {
	printk("Error: PWM device %s is not ready\n", pwm0_dev->name);
	return;
    }
    else  {
        printk("PWM device %s is ready\n", pwm0_dev->name);            
    }

    while(1) {
        k_sem_take(&sem_bc, K_FOREVER);

        printk("Atribuir valor a LED: %d (Thread C)\n", DadosBC2);

        ret = pwm_pin_set_usec(pwm0_dev, pwm0_channel, pwmPeriod_us,(unsigned int)((pwmPeriod_us*DadosBC)/100), PWM_POLARITY_NORMAL);
        if (ret) {
          printk("Error %d: failed to set pulse width\n", ret);
          return;
        }    
    }
}

void thread_D_code(void *argA , void *argB, void *argC)
{
    /* Other variables */
    long int nact = 0;
    int err = 0;

    /* Thread loop */
    while(1) {
        k_sem_take(&sem_ad,  K_FOREVER);

        printk("Leitura 10 amostras (Thread D)\n");

        for(int i = 0; i < 10; i++){
          err=adc_sample();
          if(err) {
            printk("adc_sample() failed with error code %d\n\r",err);
          }
          else {
            if(adc_sample_buffer[0] > 1023) {
                printk("adc reading out of range\n\r");
                adc_sample_buffer[0] = 0;
            }
          }
          DadosDE[i] = adc_sample_buffer[0];
          printk("%d ", DadosDE[i]); 
        }
        k_sem_give(&sem_de);
    }
}

void thread_E_code(void *argA , void *argB, void *argC)
{
    long int nact = 0;

    while(1) {
        k_sem_take(&sem_de,  K_FOREVER);

        int avg = 0;
        int cnt = 0;
        int avgmax = 0;
        int avgmin = 0;
        int sum = 0;

        printk("\nCalculo do valor final (Thread E)\n");
        for(int i = 0; i < 10; i++){
          avg += DadosDE[i];
        }
        avg = avg/10;

        avgmax = avg + avg*0.1;
        avgmin = avg - avg*0.1;

        for(int i = 0; i < 10; i++){
          if(DadosDE[i] < avgmax || DadosDE[i] > avgmin) {
            sum += DadosDE[i];
            cnt++;
          }
        }

        DadosEF = sum/cnt;
        
        k_sem_give(&sem_ef);
    }
}

void thread_F_code(void *argA , void *argB, void *argC)
{
    /* Other variables */
    const struct device *pwm0_dev;          /* Pointer to PWM device structure */
    int pwm0_channel  = 13;                 /* Ouput pin associated to pwm channel. See DTS for pwm channel - output pin association */
     
    unsigned int pwmPeriod_us = 1000;       /* PWM period in us */
    int ret = 0;
    long int nact = 0;

    pwm0_dev = device_get_binding(DT_LABEL(PWM0_NID));
    if (pwm0_dev == NULL) {
	printk("Error: PWM device %s is not ready\n", pwm0_dev->name);
	return;
    }
    else  {
        printk("PWM device %s is ready\n", pwm0_dev->name);            
    }

    while(1) {
        k_sem_take(&sem_ef, K_FOREVER);

        printk("Atribuir valor a LED: %d (Thread F)\n", DadosEF);

        ret = pwm_pin_set_usec(pwm0_dev, pwm0_channel, pwmPeriod_us,(unsigned int)((pwmPeriod_us*DadosEF)/1023), PWM_POLARITY_NORMAL);
        if (ret) {
          printk("Error %d: failed to set pulse width\n", ret);
          return;
        }    
    }
}
