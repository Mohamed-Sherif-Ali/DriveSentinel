#include <Adafruit_Fingerprint.h>
#include <HardwareSerial.h>

HardwareSerial mySerial(2);  // RX = GPIO16, TX = GPIO17
Adafruit_Fingerprint finger(&mySerial);

// Store fingerprint names in an array (Max 127 fingerprints)
String fingerprintNames[128];

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing Fingerprint Sensor...");

  mySerial.begin(57600, SERIAL_8N1, 16, 17); // RX=16, TX=17

  if (finger.verifyPassword()) {
    Serial.println("✅ Fingerprint sensor initialized successfully!");
  } else {
    Serial.println("❌ Fingerprint sensor not detected. Check connections.");
    while (1); // Halt if the sensor is not found
  }
}

void loop() {
  Serial.println("\nChoose an option:");
  Serial.println("1. Enroll a fingerprint");
  Serial.println("2. Verify a fingerprint");
  Serial.println("3. List stored fingerprints");
  Serial.println("4. Delete a fingerprint");
  Serial.print("Enter 1, 2, 3, or 4: ");

  while (!Serial.available());  // Wait for user input
  String option = Serial.readStringUntil('\n');  // Read full input
  option.trim();  // Remove spaces and newlines

  if (option == "1") {
    enrollFingerprint();
  } else if (option == "2") {
    verifyFingerprint();
  } else if (option == "3") {
    listStoredFingerprints();
  } else if (option == "4") {
    deleteFingerprint();
  } else {
    Serial.println("❌ Invalid option. Please enter 1, 2, 3, or 4.");
  }
}

void enrollFingerprint() {
  Serial.println("\nEnter a name/ID for the new fingerprint and press Enter: ");
  while (!Serial.available());
  String id = Serial.readStringUntil('\n');
  id.trim();

  if (id.length() == 0) {
    Serial.println("❌ Invalid ID. Please enter a valid name.");
    return;
  }

  int emptySlot = findEmptySlot();
  if (emptySlot == -1) {
    Serial.println("❌ No available slots for new fingerprints.");
    return;
  }

  Serial.print("Enrolling fingerprint for ");
  Serial.println(id);

  waitForUser("Press Enter to start enrollment...");

  int result = captureFingerprint(1);
  if (result != FINGERPRINT_OK) return;

  Serial.println("Remove your finger.");
  waitForUser("Press Enter to proceed to the second scan...");

  result = captureFingerprint(2);
  if (result != FINGERPRINT_OK) return;

  result = finger.createModel();
  if (result == FINGERPRINT_OK) {
    Serial.println("✅ Fingerprints matched!");
    result = finger.storeModel(emptySlot);
    if (result == FINGERPRINT_OK) {
      fingerprintNames[emptySlot] = id;
      Serial.println("✅ Fingerprint successfully stored!");
    } else {
      Serial.println("❌ Error storing fingerprint.");
    }
  } else {
    Serial.println("❌ Fingerprints did not match.");
  }
}

void verifyFingerprint() {
  Serial.println("\nPress Enter to verify fingerprint...");
  waitForUser("");

  Serial.println("Place your finger on the sensor...");
  int result = finger.getImage();
  if (result == FINGERPRINT_NOFINGER) {
    Serial.println("❌ No finger detected.");
    return;
  } else if (result != FINGERPRINT_OK) {
    Serial.println("❌ Error capturing fingerprint.");
    return;
  }

  result = finger.image2Tz();
  if (result != FINGERPRINT_OK) {
    Serial.println("❌ Error converting fingerprint.");
    return;
  }

  result = finger.fingerFastSearch();
  if (result == FINGERPRINT_OK) {
    Serial.println("✅ Fingerprint matched!");
    Serial.print("ID: ");
    Serial.println(fingerprintNames[finger.fingerID]);
    Serial.print("Confidence: ");
    Serial.println(finger.confidence);
  } else {
    Serial.println("❌ No match found.");
  }
}

void listStoredFingerprints() {
  Serial.println("\nStored Fingerprints:");
  bool found = false;
  for (int i = 0; i < 128; i++) {
    if (fingerprintNames[i] != "") {
      Serial.print("ID ");
      Serial.print(i);
      Serial.print(": ");
      Serial.println(fingerprintNames[i]);
      found = true;
    }
  }
  if (!found) {
    Serial.println("No fingerprints stored.");
  }
}

void deleteFingerprint() {
  Serial.println("\nEnter the name/ID of the fingerprint to delete: ");
  while (!Serial.available());
  String id = Serial.readStringUntil('\n');
  id.trim();

  int slot = findFingerprintByName(id);
  if (slot == -1) {
    Serial.println("❌ No fingerprint found with that name.");
    return;
  }

  if (finger.deleteModel(slot) == FINGERPRINT_OK) {
    Serial.println("✅ Fingerprint deleted successfully.");
    fingerprintNames[slot] = "";
  } else {
    Serial.println("❌ Error deleting fingerprint.");
  }
}

// Helper function to capture fingerprint images
int captureFingerprint(int slot) {
  int result;
  Serial.println("Place your finger on the sensor...");
  while (true) {
    result = finger.getImage();
    if (result == FINGERPRINT_NOFINGER) {
      Serial.println("No finger detected. Try again.");
      delay(500);
    } else if (result == FINGERPRINT_PACKETRECIEVEERR) {
      Serial.println("❌ Communication error.");
      return -1;
    } else if (result == FINGERPRINT_IMAGEFAIL) {
      Serial.println("❌ Imaging error.");
      return -1;
    } else if (result == FINGERPRINT_OK) {
      break;
    }
  }

  result = finger.image2Tz(slot);
  if (result != FINGERPRINT_OK) {
    Serial.println("❌ Error converting image.");
    return -1;
  }
  return FINGERPRINT_OK;
}

// Find the first empty fingerprint slot
int findEmptySlot() {
  for (int i = 1; i < 128; i++) {
    if (fingerprintNames[i] == "") return i;
  }
  return -1;
}

// Find a fingerprint by name
int findFingerprintByName(String id) {
  for (int i = 1; i < 128; i++) {
    if (fingerprintNames[i] == id) return i;
  }
  return -1;
}

// Helper function to wait for user input
void waitForUser(String message) {
  if (message.length() > 0) {
    Serial.println(message);
  }
  while (!Serial.available());
  while (Serial.available()) Serial.read();  // Clear input buffer
}
