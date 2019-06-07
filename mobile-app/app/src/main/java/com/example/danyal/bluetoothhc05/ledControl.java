package com.example.danyal.bluetoothhc05;

import android.app.ProgressDialog;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.content.Context;
import android.content.Intent;
import android.os.AsyncTask;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.telephony.TelephonyManager;
import android.view.View;
import android.widget.Button;
import android.widget.CompoundButton;
import android.widget.EditText;
import android.widget.ImageButton;
import android.widget.Switch;
import android.widget.TextView;
import android.widget.Toast;

import com.google.android.gms.common.util.Hex;
import com.google.firebase.database.DatabaseReference;
import com.google.firebase.database.FirebaseDatabase;

import java.io.IOException;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Calendar;
import java.util.UUID;

public class ledControl extends AppCompatActivity {

    Button btn1, btn2, btn3, btn4, btn5, btnDis;
    String address = null;
    TextView lumn;
    private ProgressDialog progress;
    BluetoothAdapter myBluetooth = null;
    BluetoothSocket btSocket = null;
    private boolean isBtConnected = false;
    static final UUID myUUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB");
    public String opt="";
    public  EditText numar;
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        Intent newint = getIntent();
        address = newint.getStringExtra(DeviceList.EXTRA_ADDRESS);

        setContentView(R.layout.activity_led_control);
        final Switch selectia = (Switch) findViewById(R.id.switchmm);
        ImageButton btnfata = (ImageButton) findViewById(R.id.fata);
        ImageButton btnstanga = (ImageButton) findViewById(R.id.stanga);
        ImageButton btndreapta = (ImageButton) findViewById(R.id.dreapta);
        ImageButton btnspate = (ImageButton) findViewById(R.id.spate);
        Button song =(Button) findViewById(R.id.song1);
        Button song2 =(Button) findViewById(R.id.song2);
        Button go =(Button) findViewById(R.id.go);
        Button stop =(Button) findViewById(R.id.stop);
        Button c =(Button) findViewById(R.id.c);
        numar = (EditText) findViewById(R.id.mm);
        final TextView mesajerul = (TextView) findViewById(R.id.textView2);
       // numar =100;
        btnDis = (Button) findViewById(R.id.disconnect);
        lumn = (TextView) findViewById(R.id.textView2);


        new ConnectBT().execute();

        selectia.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                // schimbam la pasi
                if (selectia.isChecked())
                {//pasi
                    selectia.setText("Pasi");
                    //opt=1;
                }
                else
                {//milimetri
                    selectia.setText("MM");
                   // opt=0;
                }
            }
        });

      /*  btn1.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick (View v) {
                sendSignal("pb"+'\r'+'\n');
            }
        });*/

        btnspate.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick (View v) {
                String mesaj="";
                opt=numar.getText().toString();
                if (selectia.toString().equals("MM")) {
                    mesaj = "forward_mm: " + opt +'\n';
                    //   mDatabase1.child(""+date).setValue(mesaj);
                }
                else
                {
                    mesaj = "forward_steps: " + opt +'\n';
                    //  mDatabase1.child(""+date).setValue(mesaj);
                }
                mesajerul.setText(mesaj);
                sendSignal(mesaj);
            }
        });

        btnstanga.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick (View v) {
                opt=numar.getText().toString();
                sendSignal("rotate_left_degrees:"+opt+'\n');
                mesajerul.setText("rotate_left_degrees:"+opt);
            }
        });

        btndreapta.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick (View v) {
                opt=numar.getText().toString();
                sendSignal("rotate_right_degrees: "+opt+'\n');
                mesajerul.setText("rotate_right_degrees: "+opt);
            }
        });
        song.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick (View v) {
                sendSignal("song1"+'\n');
                mesajerul.setText("song1: ");
            }
        });
        song2.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick (View v) {
                sendSignal("song2"+'\n');
                mesajerul.setText("song2: ");
            }
        });
        go.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick (View v) {
                sendSignal("go"+'\n');
            }
        });
        stop.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick (View v) {
                sendSignal("st"+'\n');
            }
        });
        c.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick (View v) {
                sendSignal("c"+'\n');
                try {
                    String m=btSocket.getInputStream().toString();
                   msg(m);
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        });
        btnfata.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick (View v) {
                String mesaj="";
                opt=numar.getText().toString();
                if (selectia.toString().equals("MM")) {
                    mesaj = "backward_mm: " + opt +'\n';
                    //   mDatabase1.child(""+date).setValue(mesaj);
                }
                else
                {
                    mesaj = "backward_steps: " + opt +'\n';
                    //  mDatabase1.child(""+date).setValue(mesaj);
                }
                sendSignal(mesaj);
               // msg(mesaj);
            }
        });

        btnDis.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick (View v) {
                Disconnect();
            }
        });
    }

    private void sendSignal ( String number ) {
        if ( btSocket != null ) {
            try {
                btSocket.getOutputStream().write(number.toString().getBytes());
            } catch (IOException e) {
                msg("Error");
            }
        }
    }

    private void Disconnect () {
        if ( btSocket!=null ) {
            try {
                btSocket.close();
            } catch(IOException e) {
                msg("Error");
            }
        }

        finish();
    }

    private void msg (String s) {
        TelephonyManager tm = (TelephonyManager) getBaseContext().getSystemService(Context.TELEPHONY_SERVICE);
        //get date/time
        DateFormat df = new SimpleDateFormat("dd MM yyyy, HH:mm");
        //send a message to database
        String date = df.format(Calendar.getInstance().getTime());
        DatabaseReference release = FirebaseDatabase.getInstance().getReference("istoric");
        release.child(""+date).setValue(""+s);
        Toast.makeText(getApplicationContext(), s, Toast.LENGTH_LONG).show();
    }

    private class ConnectBT extends AsyncTask<Void, Void, Void> {
        private boolean ConnectSuccess = true;

        @Override
        protected  void onPreExecute () {
            progress = ProgressDialog.show(ledControl.this, "Connecting...", "Please Wait!!!");
        }

        @Override
        protected Void doInBackground (Void... devices) {
            try {
                if ( btSocket==null || !isBtConnected ) {
                    myBluetooth = BluetoothAdapter.getDefaultAdapter();
                    BluetoothDevice dispositivo = myBluetooth.getRemoteDevice(address);
                    btSocket = dispositivo.createInsecureRfcommSocketToServiceRecord(myUUID);
                    BluetoothAdapter.getDefaultAdapter().cancelDiscovery();
                    btSocket.connect();
                }
            } catch (IOException e) {
                ConnectSuccess = false;
            }

            return null;
        }

        @Override
        protected void onPostExecute (Void result) {
            super.onPostExecute(result);

            if (!ConnectSuccess) {
                msg("Connection Failed. Is it a SPP Bluetooth? Try again.");
                finish();
            } else {
                msg("Connected");
                isBtConnected = true;
            }

            progress.dismiss();
        }
    }
}
