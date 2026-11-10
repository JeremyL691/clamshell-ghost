CC      := clang
CFLAGS  := -O2 -Wall -Wextra -fobjc-arc
LDFLAGS := -framework Foundation -framework AppKit -framework CoreGraphics -framework IOKit -framework CoreAudio -framework AudioToolbox

TARGET  := esp32-smart-dummy
BIN_DIR := $(HOME)/.local/bin
INST_BIN:= $(BIN_DIR)/$(TARGET)
LOG_TMP := /tmp/esp32-smart-dummy.log
SRC     := esp32_smart_dummy.m
PLIST   := com.waveshare.esp32-smart-dummy.plist
AGENT   := $(HOME)/Library/LaunchAgents/$(PLIST)

.PHONY: all clean status install install-standalone uninstall

all: $(TARGET)

$(TARGET): $(SRC)
	$(CC) $(CFLAGS) $(LDFLAGS) $(SRC) -o $(TARGET)
	codesign -f -s - --identifier com.waveshare.esp32-smart-dummy $(TARGET)

status: $(TARGET)
	./$(TARGET) --status

install: $(TARGET)
	mkdir -p $(BIN_DIR) $(HOME)/Library/LaunchAgents
	rm -f $(INST_BIN)
	cp $(TARGET) $(INST_BIN)
	codesign -f -s - --identifier com.waveshare.esp32-smart-dummy $(INST_BIN)
	if [ -f smart-dummy.log ] && [ ! -L smart-dummy.log ]; then cat smart-dummy.log >> $(LOG_TMP) 2>/dev/null || true; rm -f smart-dummy.log; fi
	ln -sf $(LOG_TMP) smart-dummy.log
	sed "s|__BIN_PATH__|$(INST_BIN)|g" $(PLIST) > $(AGENT)
	$(INST_BIN) --hold-awake 4 >/dev/null 2>&1 &
	-launchctl bootout gui/$$(id -u)/com.waveshare.esp32-smart-dummy 2>/dev/null
	for i in $$(seq 1 40); do launchctl print gui/$$(id -u)/com.waveshare.esp32-smart-dummy >/dev/null 2>&1 || break; sleep 0.05; done
	launchctl bootstrap gui/$$(id -u) $(AGENT)

install-standalone: $(TARGET)
	mkdir -p $(BIN_DIR) $(HOME)/Library/LaunchAgents
	rm -f $(INST_BIN)
	cp $(TARGET) $(INST_BIN)
	codesign -f -s - --identifier com.waveshare.esp32-smart-dummy $(INST_BIN)
	if [ -f smart-dummy.log ] && [ ! -L smart-dummy.log ]; then cat smart-dummy.log >> $(LOG_TMP) 2>/dev/null || true; rm -f smart-dummy.log; fi
	ln -sf $(LOG_TMP) smart-dummy.log
	sed "s|<string>__BIN_PATH__</string>|<string>$(INST_BIN)</string><string>--standalone</string>|g" $(PLIST) > $(AGENT)
	$(INST_BIN) --hold-awake 4 >/dev/null 2>&1 &
	-launchctl bootout gui/$$(id -u)/com.waveshare.esp32-smart-dummy 2>/dev/null
	for i in $$(seq 1 40); do launchctl print gui/$$(id -u)/com.waveshare.esp32-smart-dummy >/dev/null 2>&1 || break; sleep 0.05; done
	launchctl bootstrap gui/$$(id -u) $(AGENT)

uninstall:
	-launchctl bootout gui/$$(id -u)/com.waveshare.esp32-smart-dummy 2>/dev/null
	rm -f $(AGENT) $(INST_BIN)

clean:
	rm -f $(TARGET)
