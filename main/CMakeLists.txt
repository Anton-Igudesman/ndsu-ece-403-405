idf_component_register(
   SRCS 
      "app/rtos_ble_led.c" 
      "ble/ble_telemetry.c"
      "dsp/audio_buffer.c" 
      "dsp/fft_engine.c" 
      "dsp/spectrum_map.c" 
      "dsp/audio_dsp_pipeline.c" 
      "led/led_effects.c" 
      "led/led_matrix_renderer.c"
      "test/self_test.c" 
      "common/app_log.c" 
   INCLUDE_DIRS 
      "."
      "app"
      "ble"
      "dsp"
      "led"
      "test"
      "common"
   REQUIRES led_strip
   PRIV_REQUIRES esp_driver_gpio esp_driver_i2s bt nvs_flash)
