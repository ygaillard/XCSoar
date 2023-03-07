/* Copyright_License {

  XCSoar Glide Computer - http://www.xcsoar.org/
  Copyright (C) 2000-2021 The XCSoar Project
  A detailed list of copyright holders can be found in the file "AUTHORS".

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
}
*/

package org.xcsoar;

import java.util.Queue;
import java.util.LinkedList;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import android.util.Log;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothGatt;
import android.bluetooth.BluetoothGattCallback;
import android.bluetooth.BluetoothGattCharacteristic;
import android.bluetooth.BluetoothGattDescriptor;
import android.bluetooth.BluetoothGattService;
import android.bluetooth.BluetoothProfile;
import android.content.Context;
import android.os.Build;

/**
 * Read Bluetooth LE sensor values and report them to a
 * #SensorListener.
 */
public final class BluetoothSensor
  extends BluetoothGattCallback
  implements AndroidSensor
{
  private static final String TAG = "XCSoar";
  private final SensorListener listener;
  private final SafeDestruct safeDestruct = new SafeDestruct();

  private final BluetoothGatt gatt;

  private int state = STATE_LIMBO;

  private BluetoothGattCharacteristic currentEnableNotification;
  private final Queue<BluetoothGattCharacteristic> enableNotificationQueue =
    new LinkedList<BluetoothGattCharacteristic>();

  private boolean haveFlytecMovement = false;
  private double flytecGroundSpeed, flytecTrack;
  private int flytecSatellites = 0;

  public BluetoothSensor(Context context, BluetoothDevice device,
                         SensorListener listener)
    throws IOException
  {
    this.listener = listener;

    if (Build.VERSION.SDK_INT >= 23)
      gatt = device.connectGatt(context, true, this, BluetoothDevice.TRANSPORT_LE);
    else
      gatt = device.connectGatt(context, true, this);

    if (gatt == null)
      throw new IOException("Bluetooth GATT connect failed");
  }

  @Override
  public void close() {
    safeDestruct.beginShutdown();
    gatt.close();
    safeDestruct.finishShutdown();
  }

  @Override
  public int getState() {
    return state;
  }

  private void setStateSafe(int _state) {
    if (_state == state)
      return;

    state = _state;

    if (safeDestruct.increment()) {
      try {
        listener.onSensorStateChanged();
      } finally {
        safeDestruct.decrement();
      }
    }
  }

  private void submitError(String msg) {
    haveFlytecMovement = false;
    flytecSatellites = 0;
    state = STATE_FAILED;

    if (safeDestruct.increment()) {
      try {
        listener.onSensorError(msg);
      } finally {
        safeDestruct.decrement();
      }
    }
  }

  private boolean doEnableNotification(BluetoothGattCharacteristic c) {
    BluetoothGattDescriptor d = c.getDescriptor(BluetoothUuids.CLIENT_CHARACTERISTIC_CONFIGURATION);
    if (d == null)
      return false;

    gatt.setCharacteristicNotification(c, true);
    d.setValue(BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE);
    return gatt.writeDescriptor(d);
  }

  private void enableNotification(BluetoothGattCharacteristic c) {
    synchronized(enableNotificationQueue) {
      if (currentEnableNotification == null) {
        currentEnableNotification = c;
        if (!doEnableNotification(c))
          currentEnableNotification = null;
      } else
        enableNotificationQueue.add(c);
    }
  }

  private void readHeartRateMeasurement(BluetoothGattCharacteristic c) {
    int offset = 0;
    final int flags = c.getIntValue(c.FORMAT_UINT8, offset);
    ++offset;

    final boolean bpm16 = (flags & 0x1) != 0;

    final int bpm = bpm16
      ? c.getIntValue(c.FORMAT_UINT16, offset)
      : c.getIntValue(c.FORMAT_UINT8, offset);

    listener.onHeartRateSensor(bpm);
  }

  static long toUnsignedLong(int x) {
    if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.N)
      // Android 7 "Nougat" supports Java 8
      return Integer.toUnsignedLong(x);

    // Reimplement for older Android versions
    long l = x;
    if (l < 0)
      l += (1L << 32);
    return l;
  }

  @Override
  public synchronized void onCharacteristicChanged(BluetoothGatt gatt,
                                                   BluetoothGattCharacteristic c) {
    if (!safeDestruct.increment())
      return;

    try {
      if (BluetoothUuids.HEART_RATE_SERVICE.equals(c.getService().getUuid())) {
        if (BluetoothUuids.HEART_RATE_MEASUREMENT_CHARACTERISTIC.equals(c.getUuid())) {
          readHeartRateMeasurement(c);
        }
      }

      //XCBK Services
      if (BluetoothUuids.XCBK_AVIATION_SERVICE.equals(c.getService().getUuid())) {
        if (BluetoothUuids.XCBK_AVIATION_ENVSENSORS_CHARACTERISTIC.equals(c.getUuid())) {
          byte [] msg =c.getValue();
          listener.onTemperature((double)ByteBuffer.wrap(msg, 0, 2).order(ByteOrder.LITTLE_ENDIAN).getShort()/100);
        }
        if (BluetoothUuids.XCBK_AVIATION_FLARM_CHARACTERISTIC.equals(c.getUuid())) {
          byte [] msg =c.getValue();
          byte[] ID = new byte[6];
          ByteBuffer.wrap(msg, 11, 6).order(ByteOrder.LITTLE_ENDIAN).get(ID, 0, ID.length);
          listener.onFlarmTraffic(
                  (int)msg[0],
                  ByteBuffer.wrap(msg, 1, 4).order(ByteOrder.LITTLE_ENDIAN).getInt(),
                  ByteBuffer.wrap(msg, 5, 4).order(ByteOrder.LITTLE_ENDIAN).getInt(),
                  ByteBuffer.wrap(msg, 9, 2).order(ByteOrder.LITTLE_ENDIAN).getShort(),
                  new String(ID, java.nio.charset.StandardCharsets.UTF_8),
                  ByteBuffer.wrap(msg, 17, 2).order(ByteOrder.LITTLE_ENDIAN).getShort(),
                  (double)ByteBuffer.wrap(msg, 19, 2).order(ByteOrder.LITTLE_ENDIAN).getShort(),
                  ByteBuffer.wrap(msg, 21, 2).order(ByteOrder.LITTLE_ENDIAN).getShort(),
                  (double)ByteBuffer.wrap(msg, 23, 2).order(ByteOrder.LITTLE_ENDIAN).getShort()/10,
                  (int)msg[25]);

        }
          if (BluetoothUuids.XCBK_AVIATION_LOCATION_CHARACTERISTIC.equals(c.getUuid())) {
          byte [] msg =c.getValue();
          float result = ByteBuffer.wrap(msg, 0, 4).order(ByteOrder.LITTLE_ENDIAN).getFloat();
          listener.onLocationSensor(0,
                  0,
                  ByteBuffer.wrap(msg, 0, 4).order(ByteOrder.LITTLE_ENDIAN).getFloat(),
                  ByteBuffer.wrap(msg, 4, 4).order(ByteOrder.LITTLE_ENDIAN).getFloat(),
                  false, true,
                  0,
                  false, 0,
                  false, 0,
                  false, 0);
        }
      }

      if (BluetoothUuids.FLYTEC_SENSBOX_SERVICE.equals(c.getService().getUuid())) {
        if (BluetoothUuids.FLYTEC_SENSBOX_NAVIGATION_SENSOR_CHARACTERISTIC.equals(c.getUuid())) {
          /* protocol documentation:
             https://github.com/flytec/SensBoxLib_iOS/blob/master/_SensBox%20Documentation/SensorBox%20BLE%20Protocol.pdf */
          final int gps_status = c.getIntValue(c.FORMAT_UINT8, 18) & 0x7;
          final boolean hasAltitude = (gps_status == 2 || gps_status == 4);

          final long time = 1000 *
            toUnsignedLong(c.getIntValue(c.FORMAT_UINT32, 0));

          listener.onLocationSensor(time,
                                    flytecSatellites,
                                    c.getIntValue(c.FORMAT_SINT32, 8) / 10000000.,
                                    c.getIntValue(c.FORMAT_SINT32, 4) / 10000000.,
                                    hasAltitude, true,
                                    c.getIntValue(c.FORMAT_SINT16, 12),
                                    haveFlytecMovement, flytecTrack,
                                    haveFlytecMovement, flytecGroundSpeed,
                                    false, 0);

          listener.onPressureAltitudeSensor(c.getIntValue(c.FORMAT_SINT16, 14));
        } else if (BluetoothUuids.FLYTEC_SENSBOX_MOVEMENT_SENSOR_CHARACTERISTIC.equals(c.getUuid())) {
          flytecGroundSpeed = c.getIntValue(c.FORMAT_SINT16, 6) / 10.;
          flytecTrack = c.getIntValue(c.FORMAT_SINT16, 8) / 10.;

          listener.onVarioSensor(c.getIntValue(c.FORMAT_SINT16, 4) / 100.f);
          listener.onAccelerationSensor1(c.getIntValue(c.FORMAT_UINT16, 16) / 10.);

          haveFlytecMovement = true;
        } else if (BluetoothUuids.FLYTEC_SENSBOX_SECOND_GPS_CHARACTERISTIC.equals(c.getUuid())) {
          flytecSatellites = c.getIntValue(c.FORMAT_UINT8, 6);
        } else if (BluetoothUuids.FLYTEC_SENSBOX_SYSTEM_CHARACTERISTIC.equals(c.getUuid())) {
          listener.onBatteryPercent(c.getIntValue(c.FORMAT_UINT8, 4));

          final double CELSIUS_OFFSET = 273.15;
          double temperatureCelsius = c.getIntValue(c.FORMAT_SINT16, 6) / 10.;
          listener.onTemperature(CELSIUS_OFFSET + temperatureCelsius);
        }
      }
    } catch (NullPointerException e) {
      /* probably caused by a malformed value - ignore */
    } finally {
      safeDestruct.decrement();
    }
  }

  @Override
  public void onConnectionStateChange(BluetoothGatt gatt,
                                      int status, int newState) {
    if (BluetoothProfile.STATE_CONNECTED == newState) {
      if (!gatt.discoverServices()) {
        submitError("Discovering GATT services request failed");
      }
    } else {
      submitError("GATT disconnected");
    }
  }

  @Override
  public void onDescriptorWrite(BluetoothGatt gatt,
                                BluetoothGattDescriptor descriptor,
                                int status) {
    synchronized(enableNotificationQueue) {
      currentEnableNotification = enableNotificationQueue.poll();
      if (currentEnableNotification != null) {
        if (!doEnableNotification(currentEnableNotification))
          currentEnableNotification = null;
      }
    }
  }

  @Override
  public void onServicesDiscovered(BluetoothGatt gatt,
                                   int status) {
    if (BluetoothGatt.GATT_SUCCESS != status) {
      submitError("Discovering GATT services failed");
      return;
    }

    boolean mtuConfirmed=false;
    int mtuRequestCounter=0;
    while (!mtuConfirmed) {
      mtuConfirmed=gatt.requestMtu(50);
      mtuRequestCounter++;
      try {
        Thread.sleep(1000);
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
    }
    Log.d(TAG, "MTU change reply received after " + mtuRequestCounter + " attempts");


    BluetoothGattService service = gatt.getService(BluetoothUuids.HEART_RATE_SERVICE);
    if (service != null) {
      BluetoothGattCharacteristic c =
        service.getCharacteristic(BluetoothUuids.HEART_RATE_MEASUREMENT_CHARACTERISTIC);
      if (c != null) {
        setStateSafe(STATE_READY);
        enableNotification(c);
      }
    }

    /* enable notifications for XCBK */
    service = gatt.getService(BluetoothUuids.XCBK_AVIATION_SERVICE);
    if (service != null) {
      BluetoothGattCharacteristic c =
              service.getCharacteristic(BluetoothUuids.XCBK_AVIATION_ATTITUDE_CHARACTERISTIC);
      if (c != null) {
        setStateSafe(STATE_READY);
        enableNotification(c);
        Log.d(TAG,"Attitude sensor found");
      }
      c = service.getCharacteristic(BluetoothUuids.XCBK_AVIATION_PRESSURE_CHARACTERISTIC);
      if (c != null) {
        setStateSafe(STATE_READY);
        enableNotification(c);
        Log.d(TAG,"Pressure sensor found");
      }
      c = service.getCharacteristic(BluetoothUuids.XCBK_AVIATION_ENVSENSORS_CHARACTERISTIC);
      if (c != null) {
        setStateSafe(STATE_READY);
        enableNotification(c);
        Log.d(TAG,"Envsensors sensor found");
      }
      c = service.getCharacteristic(BluetoothUuids.XCBK_AVIATION_FLARM_CHARACTERISTIC);
      if (c != null) {
        setStateSafe(STATE_READY);
        enableNotification(c);
        Log.d(TAG,"Flarm found");
      }
      c = service.getCharacteristic(BluetoothUuids.XCBK_AVIATION_LOCATION_CHARACTERISTIC);
      if (c != null) {
        setStateSafe(STATE_READY);
        enableNotification(c);
        Log.d(TAG,"Location sensor found");
      }
    }


    /* enable notifications for Flytec Sensbox */
    service = gatt.getService(BluetoothUuids.FLYTEC_SENSBOX_SERVICE);
    if (service != null) {
      BluetoothGattCharacteristic c =
        service.getCharacteristic(BluetoothUuids.FLYTEC_SENSBOX_NAVIGATION_SENSOR_CHARACTERISTIC);
      if (c != null) {
        setStateSafe(STATE_READY);
        enableNotification(c);
      }

      c = service.getCharacteristic(BluetoothUuids.FLYTEC_SENSBOX_MOVEMENT_SENSOR_CHARACTERISTIC);
      if (c != null)
        enableNotification(c);

      c = service.getCharacteristic(BluetoothUuids.FLYTEC_SENSBOX_SECOND_GPS_CHARACTERISTIC);
      if (c != null)
        enableNotification(c);

      c = service.getCharacteristic(BluetoothUuids.FLYTEC_SENSBOX_SYSTEM_CHARACTERISTIC);
      if (c != null)
        enableNotification(c);
    }

    if (state == STATE_LIMBO)
      submitError("Unsupported Bluetooth device");
  }
}
