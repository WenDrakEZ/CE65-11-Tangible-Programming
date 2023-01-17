#include <lvgl.h>
#include "SPI.h"
#include "TFT_eSPI.h"
#include <Ticker.h>
#include "SD.h"
#include "FS.h"
#include "Audio.h"

Ticker tick;
#define LVGL_TICK_PERIOD 1

// Define I2S connections
#define I2S_DOUT  22
#define I2S_BCLK  26
#define I2S_LRC   25

// Define SD card connections
#define SD_CS    15
#define SPI_MOSI 23
#define SPI_MISO 19
#define SPI_SCK  18

Audio audio;

uint16_t calData[5] = { 410, 3443, 269, 3364, 7 };

//display buffer
static lv_disp_buf_t disp_buf;
static lv_color_t buf[LV_HOR_RES_MAX * 10];

TFT_eSPI tft = TFT_eSPI();

// ใช้งานร่วม Driver TFT_sSPI
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
  uint16_t c;

  tft.startWrite(); /* Start new TFT transaction */
  tft.setAddrWindow(area->x1, area->y1, (area->x2 - area->x1 + 1), (area->y2 - area->y1 + 1)); /* set the working window */
  for (int y = area->y1; y <= area->y2; y++) {
    for (int x = area->x1; x <= area->x2; x++) {
      c = color_p->full;
      tft.writeColor(c, 1);
      color_p++;
    }
  }
  tft.endWrite(); /* terminate TFT transaction */
  lv_disp_flush_ready(disp); /* tell lvgl that flushing is done */
}

//ใช้งานร่วม Driver TFT_sSPI
bool my_input_read(lv_indev_drv_t * indev, lv_indev_data_t * data) {
  static uint16_t prev_x, prev_y;
  uint16_t X = 0, Y = 0;
  data -> state = tft.getTouch(&X,&Y) ? LV_INDEV_STATE_PR : LV_INDEV_STATE_REL;
  if(data -> state == LV_INDEV_STATE_PR) {
    prev_x = X;
    prev_y = Y;
  }
  data -> point.x = prev_x;
  data -> point.y = prev_y;
  return false;
}


void setup() {
  Serial.begin(9600);

  lv_init();
  tft.begin();
  tft.setRotation(1);
  tft.setTouch(calData);

  //SD card
  pinMode(SD_CS,OUTPUT);
  digitalWrite(SD_CS,HIGH);
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  Serial.begin(115200);
  if(!SD.begin(SD_CS)){
    Serial.println("Error talking to SD card!");
    while(true);
  }

  //display interface
  lv_disp_buf_init(&disp_buf, buf, NULL, LV_HOR_RES_MAX *10);
  lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = 320;
  disp_drv.ver_res = 240;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.buffer = &disp_buf;
  lv_disp_drv_register(&disp_drv);
  
  //input device interface
  lv_indev_drv_t indev_drv;
  lv_indev_drv_init (&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_input_read;
  lv_indev_drv_register(&indev_drv);
  
  // I2S amplifier
  audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
  audio.setVolume(10);

  delay(500);

  create_object();

  tick.attach_ms(LVGL_TICK_PERIOD,lv_tick_handler);
}

void loop(void) {
  lv_task_handler();
  delay(5);

  audio.loop();
}

static void lv_tick_handler(void){
  lv_tick_inc(LVGL_TICK_PERIOD);
}

static lv_style_t style_btn;
static lv_anim_path_t path;

lv_obj_t * img;
LV_IMG_DECLARE(start);
LV_IMG_DECLARE(finish);
LV_IMG_DECLARE(sword_small);
LV_IMG_DECLARE(sword_large);
LV_IMG_DECLARE(vs_monster);
LV_IMG_DECLARE(front);
LV_IMG_DECLARE(left);
LV_IMG_DECLARE(right);
LV_IMG_DECLARE(back);
LV_IMG_DECLARE(pick);

void create_object(){

    lv_style_init(&style_btn);
    lv_style_set_radius(&style_btn, LV_STATE_DEFAULT, 10);
    lv_style_set_bg_opa(&style_btn, LV_STATE_DEFAULT, LV_OPA_COVER);
    lv_style_set_bg_color(&style_btn, LV_STATE_DEFAULT, LV_COLOR_SILVER);
    lv_style_set_bg_grad_color(&style_btn, LV_STATE_DEFAULT, LV_COLOR_GRAY);
    lv_style_set_bg_grad_dir(&style_btn, LV_STATE_DEFAULT, LV_GRAD_DIR_VER);

    /*Swap the colors in pressed state*/
    lv_style_set_bg_color(&style_btn, LV_STATE_PRESSED, LV_COLOR_GRAY);
    lv_style_set_bg_grad_color(&style_btn, LV_STATE_PRESSED, LV_COLOR_SILVER);

    /*Add a border*/
    lv_style_set_border_color(&style_btn, LV_STATE_DEFAULT, LV_COLOR_WHITE);
    lv_style_set_border_opa(&style_btn, LV_STATE_DEFAULT, LV_OPA_70);
    lv_style_set_border_width(&style_btn, LV_STATE_DEFAULT, 2);

    lv_style_init(&style_btn);
    lv_style_set_bg_color(&style_btn, LV_STATE_DEFAULT, LV_COLOR_BLACK);
    lv_style_set_bg_grad_color(&style_btn, LV_STATE_DEFAULT, LV_COLOR_MAROON);
    lv_style_set_bg_color(&style_btn, LV_STATE_PRESSED, LV_COLOR_MAROON);
    lv_style_set_bg_grad_color(&style_btn, LV_STATE_PRESSED, LV_COLOR_RED);
    lv_style_set_text_color(&style_btn, LV_STATE_DEFAULT, LV_COLOR_WHITE);

    
    lv_anim_path_init(&path);
    lv_anim_path_set_cb(&path, lv_anim_path_overshoot);
    lv_style_set_transition_prop_1(&style_btn, LV_STATE_DEFAULT, LV_STYLE_TRANSFORM_HEIGHT);
    lv_style_set_transition_prop_2(&style_btn, LV_STATE_DEFAULT, LV_STYLE_TRANSFORM_WIDTH);
    lv_style_set_transition_time(&style_btn, LV_STATE_DEFAULT, 300);
    lv_style_set_transition_path(&style_btn, LV_STATE_DEFAULT, &path);

    lv_style_set_transform_height(&style_btn, LV_STATE_PRESSED, -5);
    lv_style_set_transform_width(&style_btn, LV_STATE_PRESSED, -10);

    //Button
    lv_obj_t * label;
    lv_obj_t * btn1 = lv_btn_create(lv_scr_act(),NULL);
    lv_obj_set_event_cb(btn1, event_tab1);
    lv_obj_set_size(btn1, 100, 50);
    lv_obj_align(btn1, NULL, LV_ALIGN_IN_BOTTOM_LEFT,5,0);
    lv_obj_add_style(btn1, LV_BTN_PART_MAIN,&style_btn);
    label = lv_label_create(btn1, NULL);
    lv_label_set_text(label,"Read");

    lv_obj_t * btn2 = lv_btn_create(lv_scr_act(),NULL);
    lv_obj_set_event_cb(btn2, event_tab2);
    lv_obj_set_size(btn2, 100, 50);
    lv_obj_align(btn2, NULL, LV_ALIGN_IN_BOTTOM_MID,0,0);
    lv_obj_add_style(btn2, LV_BTN_PART_MAIN,&style_btn);
    label = lv_label_create(btn2, NULL);
    lv_label_set_text(label,"Play");

    lv_obj_t * btn3 = lv_btn_create(lv_scr_act(),NULL);
    lv_obj_set_event_cb(btn3, event_tab3);
    lv_obj_set_size(btn3, 100, 50);
    lv_obj_align(btn3, NULL, LV_ALIGN_IN_BOTTOM_RIGHT,-5,0);
    lv_obj_add_style(btn3, LV_BTN_PART_MAIN,&style_btn);
    label = lv_label_create(btn3, NULL);
    lv_label_set_text(label,"Stop");

}

//Button event
static void event_tab1(lv_obj_t * obj, lv_event_t tab1){
  if(tab1 == LV_EVENT_CLICKED){
    audio.connecttoFS(SD,"/MUSIC2.mp3");
  }else{
  }
}

static void event_tab2(lv_obj_t * obj, lv_event_t tab2){
  if (tab2 == LV_EVENT_CLICKED){
   lv_sword_large();
  }else{
  }
}

static void event_tab3(lv_obj_t * obj, lv_event_t tab3){
  if (tab3 == LV_EVENT_CLICKED){
    lv_obj_del(img);
  }else{
  }
}

void lv_sword_small(){
  img = lv_img_create(lv_scr_act(), NULL);
  lv_img_set_src(img, &sword_small);
  lv_obj_set_size(img, 50, 50);
  lv_obj_align(img,NULL, LV_ALIGN_IN_TOP_RIGHT, 0, 0);
}

void lv_sword_large(){
  img = lv_img_create(lv_scr_act(), NULL);
  lv_img_set_src(img, &sword_large);
  lv_obj_set_size(img, 120, 120);
  lv_obj_align(img,NULL, LV_ALIGN_CENTER, 0, 0);
}

void lv_vs_monster(){
  img = lv_img_create(lv_scr_act(), NULL);
  lv_img_set_src(img, &vs_monster);
  lv_obj_set_size(img, 120, 120);
  lv_obj_align(img,NULL, LV_ALIGN_CENTER, 0, -20);
}

void lv_front(){
  img = lv_img_create(lv_scr_act(), NULL);
  lv_img_set_src(img, &front);
  lv_obj_set_size(img, 80, 120);
  lv_obj_align(img,NULL, LV_ALIGN_CENTER, 0, -20);
}

void lv_left(){
  img = lv_img_create(lv_scr_act(), NULL);
  lv_img_set_src(img, &left);
  lv_obj_set_size(img, 120, 80);
  lv_obj_align(img,NULL, LV_ALIGN_CENTER, 0, 0);
}

void lv_right(){
  img = lv_img_create(lv_scr_act(), NULL);
  lv_img_set_src(img, &right);
  lv_obj_set_size(img, 120, 80);
  lv_obj_align(img,NULL, LV_ALIGN_CENTER, 0, 0);
}

void lv_back(){
  img = lv_img_create(lv_scr_act(), NULL);
  lv_img_set_src(img, &back);
  lv_obj_set_size(img, 80, 120);
  lv_obj_align(img,NULL, LV_ALIGN_CENTER, 0, -20);
}

void lv_start(){
  img = lv_img_create(lv_scr_act(), NULL);
  lv_img_set_src(img, &start);
  lv_obj_set_size(img, 80, 120);
  lv_obj_align(img,NULL, LV_ALIGN_CENTER, 0, -20);
}

void lv_finish(){
  img = lv_img_create(lv_scr_act(), NULL);
  lv_img_set_src(img, &finish);
  lv_obj_set_size(img, 80, 120);
  lv_obj_align(img,NULL, LV_ALIGN_CENTER, 0, -20);
}

void lv_pick(){
  img = lv_img_create(lv_scr_act(), NULL);
  lv_img_set_src(img, &pick);
  lv_obj_set_size(img, 80, 120);
  lv_obj_align(img,NULL, LV_ALIGN_CENTER, 0, -20);
}

void lv_ex_get_started_2(void)
{
    static lv_style_t style_btn;
    static lv_style_t style_btn_red;

    /*Create a simple button style*/
    lv_style_init(&style_btn);
    lv_style_set_radius(&style_btn, LV_STATE_DEFAULT, 10);
    lv_style_set_bg_opa(&style_btn, LV_STATE_DEFAULT, LV_OPA_COVER);
    lv_style_set_bg_color(&style_btn, LV_STATE_DEFAULT, LV_COLOR_SILVER);
    lv_style_set_bg_grad_color(&style_btn, LV_STATE_DEFAULT, LV_COLOR_GRAY);
    lv_style_set_bg_grad_dir(&style_btn, LV_STATE_DEFAULT, LV_GRAD_DIR_VER);

    /*Swap the colors in pressed state*/
    lv_style_set_bg_color(&style_btn, LV_STATE_PRESSED, LV_COLOR_GRAY);
    lv_style_set_bg_grad_color(&style_btn, LV_STATE_PRESSED, LV_COLOR_SILVER);

    /*Add a border*/
    lv_style_set_border_color(&style_btn, LV_STATE_DEFAULT, LV_COLOR_WHITE);
    lv_style_set_border_opa(&style_btn, LV_STATE_DEFAULT, LV_OPA_70);
    lv_style_set_border_width(&style_btn, LV_STATE_DEFAULT, 2);

    /*Different border color in focused state*/
    lv_style_set_border_color(&style_btn, LV_STATE_FOCUSED, LV_COLOR_BLUE);
    lv_style_set_border_color(&style_btn, LV_STATE_FOCUSED | LV_STATE_PRESSED, LV_COLOR_NAVY);

    /*Set the text style*/
    lv_style_set_text_color(&style_btn, LV_STATE_DEFAULT, LV_COLOR_WHITE);

    /*Make the button smaller when pressed*/
    lv_style_set_transform_height(&style_btn, LV_STATE_PRESSED, -5);
    lv_style_set_transform_width(&style_btn, LV_STATE_PRESSED, -10);
#if LV_USE_ANIMATION
    /*Add a transition to the size change*/
    static lv_anim_path_t path;
    lv_anim_path_init(&path);
    lv_anim_path_set_cb(&path, lv_anim_path_overshoot);

    lv_style_set_transition_prop_1(&style_btn, LV_STATE_DEFAULT, LV_STYLE_TRANSFORM_HEIGHT);
    lv_style_set_transition_prop_2(&style_btn, LV_STATE_DEFAULT, LV_STYLE_TRANSFORM_WIDTH);
    lv_style_set_transition_time(&style_btn, LV_STATE_DEFAULT, 300);
    lv_style_set_transition_path(&style_btn, LV_STATE_DEFAULT, &path);
#endif

    /*Create a red style. Change only some colors.*/
    lv_style_init(&style_btn_red);
    lv_style_set_bg_color(&style_btn_red, LV_STATE_DEFAULT, LV_COLOR_RED);
    lv_style_set_bg_grad_color(&style_btn_red, LV_STATE_DEFAULT, LV_COLOR_MAROON);
    lv_style_set_bg_color(&style_btn_red, LV_STATE_PRESSED, LV_COLOR_MAROON);
    lv_style_set_bg_grad_color(&style_btn_red, LV_STATE_PRESSED, LV_COLOR_RED);
    lv_style_set_text_color(&style_btn_red, LV_STATE_DEFAULT, LV_COLOR_WHITE);
#if LV_USE_BTN
    /*Create buttons and use the new styles*/
    lv_obj_t * btn = lv_btn_create(lv_scr_act(), NULL);     /*Add a button the current screen*/
    lv_obj_set_pos(btn, 10, 10);                            /*Set its position*/
    lv_obj_set_size(btn, 120, 50);                          /*Set its size*/
    lv_obj_reset_style_list(btn, LV_BTN_PART_MAIN);         /*Remove the styles coming from the theme*/
    lv_obj_add_style(btn, LV_BTN_PART_MAIN, &style_btn);

    lv_obj_t * label = lv_label_create(btn, NULL);          /*Add a label to the button*/
    lv_label_set_text(label, "Button");                     /*Set the labels text*/

    /*Create a new button*/
    lv_obj_t * btn2 = lv_btn_create(lv_scr_act(), btn);
    lv_obj_set_pos(btn2, 10, 80);
    lv_obj_set_size(btn2, 120, 50);                             /*Set its size*/
    lv_obj_reset_style_list(btn2, LV_BTN_PART_MAIN);         /*Remove the styles coming from the theme*/
    lv_obj_add_style(btn2, LV_BTN_PART_MAIN, &style_btn);
    lv_obj_add_style(btn2, LV_BTN_PART_MAIN, &style_btn_red);   /*Add the red style on top of the current */
    lv_obj_set_style_local_radius(btn2, LV_BTN_PART_MAIN, LV_STATE_DEFAULT, LV_RADIUS_CIRCLE); /*Add a local style*/

    label = lv_label_create(btn2, NULL);          /*Add a label to the button*/
    lv_label_set_text(label, "Button 2");                     /*Set the labels text*/
#endif
}