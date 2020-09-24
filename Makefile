#Change LOGFOLDER when invoking commandline make to change where data logs are stored
LOGFOLDER ?= "sample1"
#Change MASTER when invoking commandline make to change the default remote link for accessing the autopilot
MASTER ?= "udpout:192.168.86.1:14551"
#Change download parameter to indicate whether we want to download PX4 logs over the mavlink.  NOTE: This is incredibly slow, and only set to '-d' if a high-speed datalink is available.
DOWNLOAD ?= "" #Set to '-d' to indicate that we want to download PX4 logs over mavlink. It is typically faster to do this by manually downloading the files from the SD card in the autopilot after running the tests.
MOTORNUM ?= 1
MOTOR1 ?= "m1"
MOTOR2 ?= "m2"
MOTOR3 ?= "m3"
MOTOR4 ?= "m4"
MOTOR5 ?= "m5"
MOTOR6 ?= "m6"


sunnyskysmall:
	mkdir -p $(LOGFOLDER)
	cd $(LOGFOLDER) && ../motor.py --master=$(MASTER) --baudrate=480600 --source-system=253 --input=../config/sunnyskysmall.txt --pwmchip=/sys/class/pwm/pwmchip0 --pwmchan=0 --motor "sunnysky_X2212-13_980kv_m$(MOTORNUM)" --runtime 15 --noramp -d

sunnysky:
	mkdir -p $(LOGFOLDER)
	cd $(LOGFOLDER) && ../motor.py --master=$(MASTER) --source-system=253 --input=../config/sunnysky.txt --pwmchip=/sys/class/pwm/pwmchip0 --pwmchan=0 --motor "sunnysky_V5208-10_340kv" --runtime 30

sunnyskyprop:
	mkdir -p $(LOGFOLDER).props
	cd $(LOGFOLDER).props && ../motor.py --master=$(MASTER) --source-system=253 --input=../config/sunnysky.txt --pwmchip=/sys/class/pwm/pwmchip0 --pwmchan=0 --motor "sunnysky_V5208-10_340kv" --runtime 30

tmotor:
	mkdir -p $(LOGFOLDER)
	cd $(LOGFOLDER) && ../motor.py --master=$(MASTER) --source-system=253 --input=../config/tmotor.txt --pwmchip=/sys/class/pwm/pwmchip0 --pwmchan=0 --motor "tmotor_mn505s_380kv" --runtime 30

tmotorprop:
	mkdir -p $(LOGFOLDER).props
	cd $(LOGFOLDER).props && ../motor.py --master=$(MASTER) --source-system=253 --input=../config/tmotor.txt --pwmchip=/sys/class/pwm/pwmchip0 --pwmchan=0 --motor "tmotor_mn505s_380kv" --runtime 30

plot_vibrations_small:
	cd $(LOGFOLDER) && ../plot_vibrations_small.py -m $(MOTOR1) -m $(MOTOR2) --motor 'sunnysky_X2212-13_980kv'

plot_vibrations_hex:
	#cd $(LOGFOLDER) && ../plot_vibrations_small.py -m m2_0deg_0stick  -m m2_0deg_1stick -m m2_90deg_1stick -m m2_180deg_1stick -m m2_270deg_1stick --model 'sunnysky_X2212-13_980kv'
	cd $(LOGFOLDER) && ../plot_vibrations_small.py -m $(MOTOR1) -m $(MOTOR2) -m $(MOTOR3) -m $(MOTOR4) -m $(MOTOR5) -m $(MOTOR6) --model 'sunnysky_X2212-13_980kv'
	#cd $(LOGFOLDER) && ../plot_vibrations_small.py -m $(MOTOR1) -m $(MOTOR2) -m $(MOTOR3) -m $(MOTOR4) -m $(MOTOR5) -m $(MOTOR6) --model 'ss'

plot_vibrations:
	cd $(LOGFOLDER) && python3 -m pdb ../plot_vibrations.py -o output.html

plot_vibrationsprop:
	cd $(LOGFOLDER).props && ../plot_vibrations.py -o output.html
