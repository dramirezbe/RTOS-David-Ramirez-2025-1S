idf_component_register(SRCS "request/http_server.c" "request/wifi_app.c" "main.c" "utils/adc_utils.c" "utils/io_utils.c" "utils/tim_ch_duty.c"
                    INCLUDE_DIRS "." "request" "utils"
                    EMBED_FILES webpage/app.css webpage/app.js webpage/favicon.ico webpage/index.html webpage/jquery-3.3.1.min.js )
