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

import android.content.Context;
import android.graphics.Color;
import android.util.AttributeSet;
import android.view.View;
import android.widget.LinearLayout;

import org.ros.android.view.RosImageView;

/**
 * Contains two sub-views to provide a simple stereo HUD.
 */
public class CardboardOverlayView extends LinearLayout {
    private static final String TAG = "CardboardOverlayView";
    private CardboardOverlayEyeView mLeftView;
    private CardboardOverlayEyeView mRightView;
    AttributeSet attrs;

    public enum Side {
        LEFT(0), RIGHT(1);

        int side;

        Side(int side) {
            this.side = side;
        }
    }

    public RosImageView getRosImageView(Side side) {
        if (mRightView == null || mLeftView == null) {
            throw new IllegalStateException("Remember to call CardboardOverlayView.setTopicInformation(String topicName, String messageType)");
        }
        return side == Side.RIGHT ? mRightView.getImageView() : mLeftView.getImageView();
    }

    public void setTopicInformation(String topicName, String messageType) {
        LayoutParams params = new LayoutParams(
                LayoutParams.MATCH_PARENT, LayoutParams.MATCH_PARENT, 1.0f);
        params.setMargins(0, 0, 0, 0);

        mLeftView = new CardboardOverlayEyeView(getContext(), attrs, topicName, messageType);
        mLeftView.setLayoutParams(params);
        addView(mLeftView);

        mRightView = new CardboardOverlayEyeView(getContext(), attrs, topicName, messageType);
        mRightView.setLayoutParams(params);
        addView(mRightView);

        // Set some reasonable defaults.
        setDepthOffset(0.016f);
        setColor(Color.rgb(150, 255, 180));
        setVisibility(View.VISIBLE);
    }

    public CardboardOverlayView(Context context, AttributeSet attrs) {
        super(context, attrs);
        this.attrs = attrs;
        setOrientation(HORIZONTAL);
    }

    private void setDepthOffset(float offset) {
        mLeftView.setOffset(offset);
        mRightView.setOffset(-offset);
    }

    private void setColor(int color) {
        mLeftView.setColor(color);
        mRightView.setColor(color);
    }
}
