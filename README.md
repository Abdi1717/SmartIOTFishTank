1. Video Presentation
Full Presentation:
https://drive.google.com/file/d/1zwMKL44WCTebq7_D3wlmGA03e1tnMMxA/view?usp=drive_link

teamperature sensor on CLI to show its working:
https://drive.google.com/file/d/1bFmTLXpeZ6VyNEjNt4AXSmczw08_qraS/view?usp=sharing

3. Project Photos & Screenshots


<img src="https://github.com/ese5160/a14-final-submission-group-intellitank/assets/24221155/e341bfcd-65a1-4019-973d-5c6debe6f3e8" width="30%"></img> <img src="https://github.com/ese5160/a14-final-submission-group-intellitank/assets/114830054/f58ab762-5a93-4765-bfa4-1c9c1ce92d29" width="30%"></img> <img src="https://github.com/ese5160/a14-final-submission-group-intellitank/assets/114830054/1af31e77-9eb8-4f6e-af04-53664f8124a2" width="30%"></img> 

<img src="https://github.com/ese5160/a14-final-submission-group-intellitank/assets/114830054/43d63a0e-0b00-469d-b319-e1435ea523b5" width="30%"></img> <img src="https://github.com/ese5160/a14-final-submission-group-intellitank/assets/114830054/88c0c3a0-8ab1-4ec9-997a-e968554df119" width="30%"></img> <img src="https://github.com/ese5160/a14-final-submission-group-intellitank/assets/114830054/28887152-c750-4a75-8aaf-4e23f81b3f96" width="30%">


</img> <img src="https://github.com/ese5160/a14-final-submission-group-intellitank/assets/24221155/8c3a776d-1528-4751-830a-a6e678bcd068" width="30%"></img> <img src="https://github.com/ese5160/a14-final-submission-group-intellitank/assets/24221155/77b6b449-84e7-47fb-8598-07eb794f7fa1" width="30%"></img> <img src="https://github.com/ese5160/a14-final-submission-group-intellitank/assets/114830054/9e56dd0b-d7ec-4f74-bc35-724c5b1fee74" width="30%"></img>

<img src="https://github.com/ese5160/a14-final-submission-group-intellitank/assets/24221155/39f4e43b-c43a-4c51-95a7-c2990955fe22" width="30%"></img> <img src="https://github.com/ese5160/a14-final-submission-group-intellitank/assets/24221155/0f2445f6-3d9b-42d3-a459-cdb194421f74" width="30%"></img> <img src="https://github.com/ese5160/a14-final-submission-group-intellitank/assets/24221155/abb466e0-90ef-4201-b4f8-ac5c9d274047" width="30%"></img>

<img src="https://github.com/ese5160/a14-final-submission-group-intellitank/assets/24221155/528a801d-f351-4e04-9258-00ba7a49b7fe" width="30%"></img> <img src="https://github.com/ese5160/a14-final-submission-group-intellitank/assets/24221155/50f7110d-6e64-4f3c-80ae-1520ae3fd1f7" width="30%"></img> <img src="https://github.com/ese5160/a14-final-submission-group-intellitank/assets/24221155/399d3bad-85a5-45b4-a13a-06dc1e7dd148" width="30%"></img> <img src="https://github.com/ese5160/a14-final-submission-group-intellitank/assets/24221155/e137ed34-d432-4d6d-87c0-5f960cfdcc86" width="30%"></img> <img src="https://github.com/ese5160/a14-final-submission-group-intellitank/assets/24221155/8cf86c1a-ad66-4c56-b863-e7b6d1ee540c" width="30%"></img> 



3. Project Summary
Consider that this output is meant to be added to your portfolio, so shorter, better quality text/graphics/graphs will be better than very verbose explanations. Grading will be done based on the quality of this effort.

	Have you ever been out and about doing something important and realize “Hey wait, I forgot to feed my fish!” If so our product is for you! Our product is a smart fish feeder which allows you to feed your fish  wirelessly fish any time and anywhere. It also gives you an estimate of the amount of food left in the feeder displayed on a state of the art LCD screen! Our full implementation would even have temperature data displayed both on the LCD screen and online for you to track.
	What inspired us to do this project was the countless fish that we had in the past who perished due to our negligence. We wanted to make sure that no one had to go through the same pain as us and make sure that feeding ones fish was just a click away. During this journey we had a couple of rough patches on how we would implement the final product, but finally came down to 4 main components: the DS18B20 temperature sensor, the Tower Pro SG92R servo motor, the US-100 ultrasonic sensor, and the SSD1306 OLED display. Our state of the art, waterproof temperature sensor was connected over  Dallas-one-wire to our SAMW25 module via gpio. The communication between the two devices was done through bitbanging the correct commands at the correct times. It’s use was to monitor the temperature of the water to make sure the fish tank neither got too hot or too cold. The servo motor was used to uncover a hole in the floor of our container which food would drop down through in order to feed the fish below,. The motor was controlled through the use of PWM waves generated by a GPIO pin connected to the SAMW25 module, this was either prompted by a command on the command line or through the node-red UI. Our ultrasonic sensor communicated over USART and was used to calculated a percentage of the casing which was empty and displayed such onto the OLED screen. As mentioned prior this system was connected to a UI implemented using node red, which communicated via MQTT.
The following are links to this instance and our PCB design:

PCB
https://upenn-eselabs.365.altium.com/designs/805853AF-C4D5-4B32-9F1A-C9FA0710EA9A?variant=%5BNo+Variations%5D&activeDocumentId=PCB1.PcbDoc&activeView=PCB&layers=%5B1,1,2,3,32,37,38,35,36,33,34,61,64,70,67108890,67108891,67108900,74,55,73%5D&location=%5B2,15.49,35.86,17.85,0%5D#design 

Node Red UI:
http://172.172.165.38:1880/ui/#!/1?socketid=2myzRMr8G0iLDx5AAAAF

	The most challenging aspect of this project was finding and implementing portable drivers for our sensors. For the Dallas One Wire protocol we ended up just writing the one-wire driver and the sensor specific driver. The scarcity of portable drivers created situations where we had to leave certain sensors in order to keep ourselves on track. Reducing the complexity of our drivers has made our code easier to both implement and read.
	We learned that multiple iterations of ideas and actual products is very important when moving through the prototyping phase as one must be completely sure about their implementation befor they put themselves to work to make it perfect. We would probably give more time to the casing and CAD to make a more sleak design.
	In orde to finish the project we would probably push data from the sensors on board to the UI through the MQTT protocol so that pet owners can know everything about their fish at all times. We would also like to improve on the integration of our product especially the physical design.

	All in all we believe that ESE 5160 was a great experience and allowed us to quickly prototype and test product oriented IOT devices. Every aspect of the class was insightful and allowed us to get a better grasp of the work cycle of product development as we move forward in our electrical engineering career.

