// SPSA functions
void write_CICLUTS(Stream &FPGA, int8_t * CICLUT){
  return; // REMOVE REMOVE REMOVE
 uint16_t temp_write;
 uint16_t register_idxs[6] = {0x76, 0x77, 0x78, 0x7A, 0x7B, 0x7C};
  for (int k=0; k<16; k++){
    for(int r=0; r<6; r++){
      temp_write = (CICLUT[12*k + 2*r + 1]<<8) | (CICLUT[12*k + 2*r + 0] & 0xff);
      reg_write(FPGA, register_idxs[r], temp_write); 
    }
  }
  return;
}

// this should be CDF normal, and then scaled by sigma and rounded.
const float randn_table[256] = {-0.560137267783445, 1.02904238477106, -0.256815645944037, -0.157790233226037, -1.11520740425788, -0.658268251257351, -1.47381041396459, 0.284312273511208, -0.552260572779522, 0.713814070077139, 1.17888888895788, 1.01489975195199, -0.124591084214391, -0.605318523527643, -1.34312468989019, 0.0325446879795542, 0.894268948491682, -0.438236445399050, -0.266486706170506, 0.542624470602307, 0.796355991362008, -1.54078164573675, 0.115998042781084, 0.111771105630039, -0.243753101714513, 0.825933334061488, -0.462070379392944, -0.935473662200686, 0.193934971920328, 0.382257863900580, -0.311765868143355, 0.726885983753274, -1.64438598397731, -1.19034205327309, -3.23689434930582, 0.445388638217526, -1.10437767520060, -0.348264218867112, 0.945289857401700, -0.0366185569274776, -0.540195088625298, -0.958304401433482, -0.335194456309518, 0.528923324077970, -1.64257340006758, -0.158925617871818, 0.712816943576050, -1.54468749791695, 0.596999034098474, 0.611212969636868, -0.992802304066036, -1.63053857289351, 0.354941891302112, 0.739318235768894, 0.504339626645250, 0.692790752188070, -1.30019861478940, 0.509370530690291, -0.683217540592728, -0.664114843872229, -1.68319484415561, 1.03847805284827, 0.0904898673275532, 0.939496333294732, 1.45941867250249, -1.25420251684747, 0.570001907295185, -0.101964916521127, 0.847939036522843, 1.04614897311356, 0.751835538774872, -0.926685079711860, -0.218497485560676, 0.181773592024582, 3.14760884564770, 1.16800732119270, 2.28903207791362, 0.335840172628465, 0.128170908056828, 0.539413498950169, -0.629695415308874, 1.18336123596773, -0.250411796541317, -1.49640006444371, 0.803445571934898, -1.69614361390219, -1.97866955433818, 0.206639542812520, -1.06327496886973, -1.02210408560197, -0.254836889160558, -0.589860556624624, -0.317171431390069, 0.189892782500697, -1.28626684367479, -1.34334533764684, -2.24019580000549, -0.488765945598957, 0.591790174126536, -0.808489806353224, 0.424813302449940, 0.464761917947336, 0.926318542676089, 0.924911782516233, -0.794735629524435, -0.966879171958318, -0.0393053192924049, -0.770155692984612, 1.39971411367949, 1.32501840679724, 0.00859505228183550, -0.526560899665478, 0.434527605753271, -0.761364116177816, -0.216992790725038, -0.632451553053171, 0.0193029982446255, 1.06731470850433, 0.671916791846532, 1.08294398201111, -1.02618453899769, 0.0953503310265970, -1.09651238584384, 0.305438473885708, 0.244847843443522, 1.23942088910222, 1.03747491336114, 0.580458472522567, -0.606431386883515, -0.576633955167144, -2.19109564797540, -0.548261155579214, -1.67837887541222, 0.863889349783098, 0.835315160950182, 0.0528112981727543, -0.476638921287456, 0.0577060256307939, 1.55577037844468, 1.29353012311594, -0.548402125094418, 0.0836788005963510, -1.23704688433752, 0.321733371905779, 0.668417083811332, 0.833373107616138, -1.65790743418869, 0.428187851437581, -0.974438319091035, -0.474740376843169, 0.718573156631743, 0.216677192120863, -0.256673808990983, 0.532820316081549, 1.58595763361936, -0.727775019493971, -1.47231829056586, -1.27982165758672, -1.05647932867793, -0.776670170599318, 0.502027099476647, -2.22614878069206, 0.152898949342728, -0.886527150768290, 1.03781558233799, -0.965574428349814, -0.0811456919132715, -1.04962066360270, -0.787274379771855, -0.355451050067281, -0.323333232012527, -0.626658280720248, 1.06728202935729, 0.474829971763341, 1.81205319496528, -0.373034006085925, 1.00407147687067, -2.23995003052527, 0.565355342936149, 0.291745481214690, -0.472306080002385, 0.623623644250937, 0.810403788842271, 0.970231104468762, 0.341264305415272, 0.527017195149670, -0.779546846588615, -1.21110364637466, -0.363112928050470, 0.102265627765526, 0.885529578995674, -0.898923170807727, -0.590450997171082, 2.95901112615965, -0.726844228311541, 1.87881724035911, 0.895985733665730, -0.660462838662538, -0.198075427184114, -0.309362545698936, 0.396288262054986, 1.31046295697035, -1.76712402037777, 1.98479626114119, 0.708099721110674, -0.651341863393624, -0.218011926955074, 0.218085895057233, 0.214592512511663, 1.14153527072830, 1.02819468618353, -0.456447966831070, -0.222024873424428, 0.662960508247404, 0.0344121649916617, 1.07549260057496, -0.792616513596948, 0.394436830467684, 0.206570536121089, 1.33732394299189, -1.46760662583129, -0.0881240246501435, -0.0340745137792967, -2.48860677561720, 1.19001543583350, 2.78968998498209, -0.326806401787327, 0.205227871354013, -0.564963484172555, 0.304915070508879, -0.478093734108158, -0.193180581182395, 1.56920654509519, -0.957034274076886, -0.678989169521118, -0.166676982474001, -0.224631634729802, 1.38595023247101, 1.99251629492159, -1.09123913760427, -0.745737703084822, 0.444434123854242, -0.478987744559655, 1.95315711720744, 1.58736306108270, 0.483928814243178, 0.488661523251807, -0.435640078933968, 0.693608703266882, 1.06638423175191, 0.0561760774491123, 1.88684167015912, 0.598303232334172, -0.201719297275794, 0.291229860730444, 1.03477508143044};

float randn(float sigma){
  return(sigma*randn_table[random(256)]);
}

float mean(float * vals, int N) {
	float total = 0.0;
	for(int i=0;i<N;i++){
		total += vals[i];
	}
	return total / N;
}

float var(float * vals, int N) {
	float total = 0.0;
	float val_mean = mean(vals, N);
	for(int i=0;i<N;i++){
		total += (vals[i]-val_mean) * (vals[i]-val_mean);
	}
	return total / N;
}

int16_t clamp(int16_t val, int16_t max) {
	val = (val > max) ? max : val;
	val = (val < -max) ? -max : val;
	return val;
}

float get_modem_metric(Stream &FPGA, int quiet){

  const uint16_t interval_count = 100;
  reg_write(FPGA, 0x67, interval_count); // interval_count * 2^16 of samples in measurement interval

  // initiate a measurement
   reg_write(FPGA, 0x68, 0);
   reg_write(FPGA, 0x68, 1); // rising flag

  // Save static past measurements (index 0,2,4,6) (can do immediately on flag)
  reg_write(FPGA, 0x69, 0); // readback mux
  uint32_t evm_acc_z1 = reg_read(FPGA, 0x68) * 65536 + reg_read(FPGA, 0x69);
  reg_write(FPGA, 0x69, 2); // readback mux
  uint32_t check_count_z1 = reg_read(FPGA, 0x68) * 65536 + reg_read(FPGA, 0x69);
  reg_write(FPGA, 0x69, 4); // readback mux
  uint32_t detect_count_z1 = reg_read(FPGA, 0x68) * 65536 + reg_read(FPGA, 0x69);
  reg_write(FPGA, 0x69, 6); // readback mux
  uint32_t sent_count_z1 = reg_read(FPGA, 0x68) * 65536 + reg_read(FPGA, 0x69);
 
  // Wait for finished
  while(reg_read(FPGA, 0x67) != 0){ // once done, resets to zero
    delay(1);
  }
  
  // New measurements (index 1,3,5,7)
  reg_write(FPGA, 0x69, 1); // readback mux
  uint32_t evm_acc = reg_read(FPGA, 0x68) * 65536 + reg_read(FPGA, 0x69);
  reg_write(FPGA, 0x69, 3); // readback mux
  uint32_t check_count = reg_read(FPGA, 0x68) * 65536 + reg_read(FPGA, 0x69);
  reg_write(FPGA, 0x69, 5); // readback mux
  uint32_t detect_count = reg_read(FPGA, 0x68) * 65536 + reg_read(FPGA, 0x69);
  reg_write(FPGA, 0x69, 7); // readback mux
  uint32_t sent_count = reg_read(FPGA, 0x68) * 65536 + reg_read(FPGA, 0x69);

  // calculate delta
    uint32_t sent_increase = (sent_count >= sent_count_z1) ? sent_count - sent_count_z1 : sent_count - sent_count_z1 + 0xffffffff;
    sent_count_z1 = sent_count;

    uint32_t detect_increase = (detect_count >= detect_count_z1) ? detect_count - detect_count_z1 : detect_count - detect_count_z1 + 0xffffffff;
    detect_count_z1 = detect_count;

    uint32_t check_increase = (check_count >= check_count_z1) ? check_count - check_count_z1 : check_count - check_count_z1 + 0xffffffff;
    check_count_z1 = check_count;

    uint32_t evm = (evm_acc >= evm_acc_z1) ? evm_acc - evm_acc_z1 : evm_acc - evm_acc_z1 + 0xffffffff;
    evm_acc_z1 = evm_acc;

 if(!quiet){
    Serial.print("checksum:");
    //if(check_increase>0)
      Serial.print(check_increase, DEC);
    Serial.print(",detects:");
    Serial.print(detect_increase, DEC);
    
    Serial.print(",sent:");
    Serial.print(sent_increase, DEC);

    //Serial.print(",EVM:"); 
    float EVM_per_frame = 100.0 * (float)evm / (float)detect_increase;
    //Serial.print(EVM_per_frame, 3); // decimal places
    Serial.print("\n");  
 }
Serial.print("((");  Serial.print(check_increase,DEC);Serial.print("))");
  return (float)check_increase;// - EVM_per_frame;//(float)10.0*check_increase + (float)1.0*detect_increase;//-1.0 * (float)evm;//(float)detect_increase;
}

void print_LUTS(int N, int8_t * CICLUT){
  char msg[256] = {0};
  for(int k=0;k<N;k++){
    sprintf(msg, "%d, ",CICLUT[k]);
    Serial.print(msg);
  }
  Serial.print("\n");
  return;
}

///// MAIN ADAPTATION //////

void spsa_adapt_CIC(Stream &FPGA, bool init_existing, int num_iters, int8_t * CICLUT, int N_LUT)
{

  //const int N_LUT = 192;
  //int8_t CICLUT[N_LUT] = {-11, 9, -57, -57, 9, -11, -11, 9, -57, -57, 9, -11, -13, 5, -23, -35, 5, 3, -13, 5, -23, -35, 5, 3, -9, 5, -25, -31, 3, -3, -9, 5, -25, -31, 3, -3, -11, 1, 9, -9, -1, 11, -11, 1, 9, -9, -1, 11, -3, 3, -31, -25, 5, -9, -3, 3, -31, -25, 5, -9, -5, -1, 3, -3, 1, 5, -5, -1, 3, -3, 1, 5, -1, -1, 1, 1, -1, -1, -1, -1, 1, 1, -1, -1, -3, -5, 35, 23, -5, 13, -3, -5, 35, 23, -5, 13, 3, 5, -35, -23, 5, -13, 3, 5, -35, -23, 5, -13, 1, 1, -1, -1, 1, 1, 1, 1, -1, -1, 1, 1, 5, 1, -3, 3, -1, -5, 5, 1, -3, 3, -1, -5, 3, -3, 31, 25, -5, 9, 3, -3, 31, 25, -5, 9, 11, -1, -9, 9, 1, -11, 11, -1, -9, 9, 1, -11, 9, -5, 25, 31, -3, 3, 9, -5, 25, 31, -3, 3, 13, -5, 23, 35, -5, -3, 13, -5, 23, 35, -5, -3, 11, -9, 57, 57, -9, 11, 11, -9, 57, 57, -9, 11};
  // adapted: -15, -3, -66, -59, 27, -4, -11, -14, -63, -64, 29, 0, -13, 1, -19, -44, 8, 6, -18, -3, -25, -39, 11, 13, -3, 2, -26, -27, 9, 8, -11, 2, -24, -31, 4, 1, -9, 0, 12, -12, -5, 13, -11, 1, 12, -13, -7, 17, -2, 11, -32, -26, 2, -8, 1, -3, -26, -28, 1, -6, -4, 6, -5, 0, 4, 10, -2, -8, 6, -4, -5, -3, -7, -3, 1, 2, -6, -3, 2, 4, -2, -1, -3, 1, -3, 2, 43, 21, -22, 16, -5, 6, 38, 22, -23, 16, 1, -1, -37, -27, 24, -21, -5, -3, -35, -27, 24, -16, 4, -5, -6, 10, -2, 1, 10, 1, -6, 3, 5, -4, -4, 6, 2, 13, -1, -2, 5, 0, -3, 6, -5, -8, 9, -2, 26, 24, -9, 14, -4, 0, 27, 23, -12, 7, 10, -5, -14, 10, 4, -11, 9, -1, -6, 5, 8, -16, 11, -6, 28, 26, -13, 2, 12, -3, 28, 36, 4, -2, 13, -5, 23, 36, -10, -5, 9, 4, 27, 37, -3, -18, 6, 6, 66, 60, -30, 8, 11, 9, 63, 60, -28, -1, 
  //int8_t CICLUT[N_LUT] = {0};

  if(init_existing) {
    Serial.print("\n Reading existing CICLUT... ");
    uint16_t temp_read;
    uint16_t register_idxs[6] = {0x76, 0x77, 0x78, 0x7A, 0x7B, 0x7C};
    for (int k=0; k<16; k++){
      for(int r=0; r<6; r++){
        temp_read = reg_read(FPGA, register_idxs[r]);
        reg_write(FPGA, register_idxs[r], temp_read); // write it back to replenish cycling shift register
        CICLUT[12*k + 2*r + 0] =  temp_read & 0xff;
        CICLUT[12*k + 2*r + 1] =  (temp_read>>8) & 0xff;
    }
    }
  }

  print_LUTS(N_LUT, CICLUT);
  write_CICLUTS(FPGA, CICLUT); // write whichever values with are initializing with

  // SPSA Parameters
  float sigma = 3.0; // perturbation scale (15 better?)
  float alpha = 2.0; // step scale (4 better?)
  float decay = 0.999; // empirical Spall result.
  const int NPOP = 50; /* perturbation population size */

  int8_t P[N_LUT][NPOP] = {0}; // Perturbations
  float R[NPOP] = {0.0}; // Rewards

  int8_t CICLUT_temp[N_LUT] = {0};
  for(int iter=0; iter<num_iters; iter++){ // Main Adaptation Loop

    // Status
    /*if(!(iter%100)){ // dump table periodically
      print_LUTS(N_LUT,CICLUT);
      Serial.print("\n");
      rx_debug_capture();
    }*/

    // Update step
    for(int n=0;n<NPOP;n++){

      for(int k=0;k<N_LUT;k++){
        // Draw perturbations
        P[k][n] = (int8_t)round(randn(sigma)); // greatly speed up by making table have rounded perturbations of degree sigma
        // Apply perturbation to base CICLUT
        CICLUT_temp[k] = (int8_t)clamp((int16_t)CICLUT[k] + P[k][n], +127);
      }

      write_CICLUTS(FPGA, CICLUT_temp);
      
      // collect reward  (wait long enough)
      do{
        R[n] = get_modem_metric(FPGA, 1);
      }  while(isnan(R[n])); // if we missed reading fast enough, it returns, so we call back to ensure fresh samples
    }
 
    // Renormalize rewards
    float mean_rew = mean(R, NPOP); //mean of reward
    float std_rew = sqrtf(var(R, NPOP)); //standard deviation of reward
    for(int k=0; k<NPOP ; k++){ 	// renormalize rewards
      R[k] = (R[k] - mean_rew) / (std_rew + 0.0001);
    }

    // Calculate update step
    for(int k=0;k<N_LUT;k++){
      float Pstep = 0.0;
      for(int n=0; n<NPOP ; n++) 
        Pstep += P[k][n] * R[n];
      Pstep = alpha * Pstep / (sigma*NPOP + 0.0001);
      CICLUT[k] = (int8_t)clamp((int16_t)round(CICLUT[k] + Pstep), +127);
    }

    // print out mean_rew (std_rew)
    char msg[256] = {0};
    sprintf (msg, "Mean Reward: %0.2f (%0.2f) \n",mean_rew, std_rew);
    Serial.print(msg);
  }

  write_CICLUTS(FPGA, CICLUT); // write final values
  return;
}