/*
 * Copyright 2014 Google Inc. All Rights Reserved.

 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package com.cloudspace.cardboard;

import android.os.Bundle;
import android.util.Log;

import com.google.vrtoolkit.cardboard.CardboardView;
import com.google.vrtoolkit.cardboard.Eye;
import com.google.vrtoolkit.cardboard.HeadTransform;
import com.google.vrtoolkit.cardboard.Viewport;

import org.ros.address.InetAddressFactory;
import org.ros.android.view.RosImageView;
import org.ros.namespace.GraphName;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;
import java.net.URI;

import javax.microedition.khronos.egl.EGLConfig;

import sensor_msgs.CompressedImage;

/**
 * A Cardboard sample application.
 */
public class CardboardViewerActivity extends RosCardboardActivity implements CardboardView.StereoRenderer {
    static final int ROS_REFRESH_PER_SEC = 1;

    private static final String TAG = "MainActivity";
    private CardboardOverlayView mOverlayView;
    private RosImageView rightRosImageView, leftRosImageView;

    public CardboardViewerActivity() {
        super("Ardrobot is running.", "Ardrobot", URI.create("http://10.100.4.65:11311"));
//        super("Cardboard", "Cardboard");
    }

    @Override
    protected void onPostCreate(Bundle savedInstanceState) {
        super.onPostCreate(savedInstanceState);
        rightRosImageView = mOverlayView.getRosImageView(CardboardOverlayView.Side.RIGHT);
        leftRosImageView = mOverlayView.getRosImageView(CardboardOverlayView.Side.LEFT);
        init(cardboardNodeMainExecutorService);
    }

    /**
     * Sets the view to our CardboardView and initializes the transformation matrices we will use
     * to render our scene.
     * //@param savedInstanceState
     */
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        setContentView(R.layout.activity_cardboard_viewer);
        CardboardView cardboardView = (CardboardView) findViewById(R.id.cardboard_view);
        cardboardView.setRenderer(this);
        setCardboardView(cardboardView);

        mOverlayView = (CardboardOverlayView) findViewById(R.id.overlay);
        mOverlayView.setTopicInformation("/camera/image/compressed", CompressedImage._TYPE);
    }

    @Override
    public void onRendererShutdown() {
        Log.i(TAG, "onRendererShutdown");
    }

    @Override
    public void onSurfaceChanged(int width, int height) {
        Log.i(TAG, "onSurfaceChanged");
    }

    /**
     * Creates the buffers we use to store information about the 3D world. OpenGL doesn't use Java
     * arrays, but rather needs data in a format it can understand. Hence we use ByteBuffers.
     *
     * @param config The EGL configuration used when creating the surface.
     */
    @Override
    public void onSurfaceCreated(EGLConfig config) {
        Log.i(TAG, "onSurfaceCreated");
    }

    /**
     * Prepares OpenGL ES before we draw a frame.
     *
     * @param headTransform The head transformation in the new frame.
     */
    @Override
    public void onNewFrame(HeadTransform headTransform) {
    }

    @Override
    public void onDrawEye(Eye eye) {

    }


    @Override
    public void onFinishFrame(Viewport viewport) {
    }

    @Override
    protected void init(NodeMainExecutor nodeMainExecutor) {
        if (rightRosImageView != null && nodeMainExecutor != null) {
            NodeConfiguration rightImageViewConfig = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostName())
                    .setMasterUri(getMasterUri());
            rightImageViewConfig.setNodeName(GraphName.of("right_eye"));
            NodeConfiguration leftImageViewConfig = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostName())
                    .setMasterUri(getMasterUri());
            leftImageViewConfig.setNodeName(GraphName.of("left_eye"));

            nodeMainExecutor.execute(rightRosImageView, rightImageViewConfig);
            nodeMainExecutor.execute(leftRosImageView, leftImageViewConfig);
        }
    }
}