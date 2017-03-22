/*
 * Copyright (C) 2011 Google Inc.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package com.cloudspace.cardboard;

import android.app.Activity;
import android.content.ComponentName;
import android.content.Intent;
import android.content.ServiceConnection;
import android.os.AsyncTask;
import android.os.IBinder;

import com.google.common.base.Preconditions;
import com.google.vrtoolkit.cardboard.CardboardActivity;

import org.ros.address.InetAddressFactory;
import org.ros.android.MasterChooser;
import org.ros.android.NodeMainExecutorService;
import org.ros.android.NodeMainExecutorServiceListener;
import org.ros.exception.RosRuntimeException;
import org.ros.node.NodeMain;
import org.ros.node.NodeMainExecutor;

import java.net.NetworkInterface;
import java.net.SocketException;
import java.net.URI;
import java.net.URISyntaxException;

/**
 * @author damonkohler@google.com (Damon Kohler)
 */
public abstract class RosCardboardActivity extends CardboardActivity {

    private static final int MASTER_CHOOSER_REQUEST_CODE = 0;

    static final String ACTION_START = "org.ros.android.ACTION_START_NODE_RUNNER_SERVICE";
    static final String ACTION_SHUTDOWN = "org.ros.android.ACTION_SHUTDOWN_NODE_RUNNER_SERVICE";
    static final String EXTRA_NOTIFICATION_TITLE = "org.ros.android.EXTRA_NOTIFICATION_TITLE";
    static final String EXTRA_NOTIFICATION_TICKER = "org.ros.android.EXTRA_NOTIFICATION_TICKER";

    private final ServiceConnection cardboardNodeMainExecutorServiceConnection;
    private final String notificationTicker;
    private final String notificationTitle;

    protected CardboardNodeMainExecutorService cardboardNodeMainExecutorService;

    private final class CardboardNodeMainExecutorServiceConnection implements ServiceConnection {

        private URI customMasterUri;

        public CardboardNodeMainExecutorServiceConnection(URI customUri) {
            super();
            customMasterUri = customUri;
        }

        @Override
        public void onServiceConnected(ComponentName name, IBinder binder) {
            cardboardNodeMainExecutorService = ((CardboardNodeMainExecutorService.CardboardLocalBinder) binder).getService();

            if (customMasterUri != null) {
                cardboardNodeMainExecutorService.setMasterUri(customMasterUri);
                cardboardNodeMainExecutorService.setRosHostname(getDefaultHostAddress());
            }
            cardboardNodeMainExecutorService.addListener(new NodeMainExecutorServiceListener() {
                @Override
                public void onShutdown(NodeMainExecutorService nodeMainExecutorService) {
                    // We may have added multiple shutdown listeners and we only want to
                    // call finish() once.
                    if (!RosCardboardActivity.this.isFinishing()) {
                        RosCardboardActivity.this.finish();
                    }
                }
            });
            if (getMasterUri() == null) {
                startMasterChooser();
            } else {
                init();
            }
        }

        @Override
        public void onServiceDisconnected(ComponentName name) {
        }
    }

    protected RosCardboardActivity(String notificationTicker, String notificationTitle) {
        this(notificationTicker, notificationTitle, null);
    }

    protected RosCardboardActivity(String notificationTicker, String notificationTitle, URI customMasterUri) {
        super();
        this.notificationTicker = notificationTicker;
        this.notificationTitle = notificationTitle;
        cardboardNodeMainExecutorServiceConnection = new CardboardNodeMainExecutorServiceConnection(customMasterUri);
    }

    @Override
    protected void onStart() {
        super.onStart();
        bindNodeMainExecutorService();
    }

    protected void bindNodeMainExecutorService() {
        Intent intent = new Intent(this, CardboardNodeMainExecutorService.class);
        intent.setAction(ACTION_START);
        intent.putExtra(EXTRA_NOTIFICATION_TICKER, notificationTicker);
        intent.putExtra(EXTRA_NOTIFICATION_TITLE, notificationTitle);
        startService(intent);
        Preconditions.checkState(
                bindService(intent, cardboardNodeMainExecutorServiceConnection, BIND_AUTO_CREATE),
                "Failed to bind NodeMainExecutorService.");
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        Intent intent = new Intent(this, NodeMainExecutorService.class);
        intent.setAction("org.ros.android.ACTION_START_NODE_RUNNER_SERVICE");
        stopService(intent);
    }

    protected void init() {
        // Run init() in a new thread as a convenience since it often requires
        // network access.
        new AsyncTask<Void, Void, Void>() {
            @Override
            protected Void doInBackground(Void... params) {
                RosCardboardActivity.this.init(cardboardNodeMainExecutorService);
                return null;
            }
        }.execute();
    }

    /**
     * This method is called in a background thread once this {@link Activity} has
     * been initialized with a master {@link URI} via the {@link MasterChooser}
     * and a {@link NodeMainExecutorService} has started. Your {@link NodeMain}s
     * should be started here using the provided {@link NodeMainExecutor}.
     *
     * @param nodeMainExecutor the {@link NodeMainExecutor} created for this {@link Activity}
     */
    protected abstract void init(NodeMainExecutor nodeMainExecutor);

    public void startMasterChooser() {
        Preconditions.checkState(getMasterUri() == null);
        // Call this method on super to avoid triggering our precondition in the
        // overridden startActivityForResult().
        super.startActivityForResult(new Intent(this, MasterChooser.class), 0);
    }

    public URI getMasterUri() {
        Preconditions.checkNotNull(cardboardNodeMainExecutorService);
        return cardboardNodeMainExecutorService.getMasterUri();
    }

    public String getRosHostname() {
        Preconditions.checkNotNull(cardboardNodeMainExecutorService);
        return cardboardNodeMainExecutorService.getRosHostname();
    }

    @Override
    public void startActivityForResult(Intent intent, int requestCode) {
        Preconditions.checkArgument(requestCode != MASTER_CHOOSER_REQUEST_CODE);
        super.startActivityForResult(intent, requestCode);
    }

    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        super.onActivityResult(requestCode, resultCode, data);
        if (resultCode == RESULT_OK) {
            if (requestCode == MASTER_CHOOSER_REQUEST_CODE) {
                String host;
                String networkInterfaceName = data.getStringExtra("ROS_MASTER_NETWORK_INTERFACE");
                // Handles the default selection and prevents possible errors
                if (networkInterfaceName == null || networkInterfaceName.equals("")) {
                    host = getDefaultHostAddress();
                } else {
                    try {
                        NetworkInterface networkInterface = NetworkInterface.getByName(networkInterfaceName);
                        host = InetAddressFactory.newNonLoopbackForNetworkInterface(networkInterface).getHostAddress();
                    } catch (SocketException e) {
                        throw new RosRuntimeException(e);
                    }
                }
                cardboardNodeMainExecutorService.setRosHostname(host);
                if (data.getBooleanExtra("ROS_MASTER_CREATE_NEW", false)) {
                    cardboardNodeMainExecutorService.startMaster(data.getBooleanExtra("ROS_MASTER_PRIVATE", true));
                } else {
                    URI uri;
                    try {
                        uri = new URI(data.getStringExtra("ROS_MASTER_URI"));
                    } catch (URISyntaxException e) {
                        throw new RosRuntimeException(e);
                    }
                    cardboardNodeMainExecutorService.setMasterUri(uri);
                }
                // Run init() in a new thread as a convenience since it often requires network access.
                new AsyncTask<Void, Void, Void>() {
                    @Override
                    protected Void doInBackground(Void... params) {
                        RosCardboardActivity.this.init(cardboardNodeMainExecutorService);
                        return null;
                    }
                }.execute();
            } else {
                // Without a master URI configured, we are in an unusable state.
                cardboardNodeMainExecutorService.forceShutdown();
            }
        }
        super.onActivityResult(requestCode, resultCode, data);
    }

    private String getDefaultHostAddress() {
        return InetAddressFactory.newNonLoopback().getHostAddress();
    }
}