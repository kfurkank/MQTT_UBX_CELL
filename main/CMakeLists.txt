idf_component_register(SRCS "Main.c" "System.c" "CellApplication.c"  "TimerTask.c" "adxl357.c" "fft.c" "../CoreMqtt/core_mqtt.c" "../CoreMqtt/core_mqtt_state.c" 
"../CoreMqtt/core_mqtt_serializer.c"   "../CoreMqtt/transport_interface.c"                

	INCLUDE_DIRS "." "../CoreMqtt/include/"
	EMBED_TXTFILES web/index.html
	EMBED_TXTFILES web/ctrlcfg.html
)