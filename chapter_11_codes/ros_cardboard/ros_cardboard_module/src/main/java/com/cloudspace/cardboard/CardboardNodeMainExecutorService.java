//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by Fernflower decompiler)
//

package com.cloudspace.cardboard;

import android.content.Intent;
import android.os.Binder;
import android.os.IBinder;

import org.ros.android.NodeMainExecutorService;
import org.ros.node.NodeMainExecutor;

public class CardboardNodeMainExecutorService extends NodeMainExecutorService implements NodeMainExecutor {
    private final IBinder cardboardBinder = new CardboardNodeMainExecutorService.CardboardLocalBinder();

    public class CardboardLocalBinder extends Binder {
        CardboardLocalBinder() {
        }

        CardboardNodeMainExecutorService getService() {
            return CardboardNodeMainExecutorService.this;
        }
    }

    @Override
    public IBinder onBind(Intent intent) {
        return this.cardboardBinder;
    }


}
