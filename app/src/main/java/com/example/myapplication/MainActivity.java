package com.example.myapplication;

import android.os.Bundle;
import android.widget.TextView;

import com.bfr.buddysdk.BuddyActivity;
import com.bfr.buddysdk.BuddySDK;

public class MainActivity extends BuddyActivity {

    private TextView textHello;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        textHello = findViewById(R.id.textHello);
    }

    // Called once the BUDDY SDK is initialised. Don't call BuddySDK before this.
    @Override
    public void onSDKReady() {
        // Pass touches on our transparent layout through to Buddy's face (BuddyCore)
        BuddySDK.UI.setViewAsFace(findViewById(R.id.view_face));

        // Tap the button: show the message on screen and have Buddy say it
        findViewById(R.id.buttonHello).setOnClickListener(view -> {
            textHello.setText("Hello from Buddy!");
            BuddySDK.Speech.startSpeaking("Hello");
        });
    }
}
