/****************************************************************************
 * apps/include/graphics/nxwidgets/cglyphsliderhorizontalgrip.hxx
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 *
 * Portions of this package derive from Woopsi (http://woopsi.org/) and
 * portions are original efforts.  It is difficult to determine at this
 * point what parts are original efforts and which parts derive from Woopsi.
 * However, in any event, the work of  Antony Dzeryn will be acknowledged
 * in most NxWidget files.  Thanks Antony!
 *
 *   Copyright (c) 2007-2011, Antony Dzeryn
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 * * Neither the names "Woopsi", "Simian Zombie" nor the
 *   names of its contributors may be used to endorse or promote products
 *   derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY Antony Dzeryn ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Antony Dzeryn BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __APPS_INCLUDE_GRAPHICS_NXWIDGETS_CGLYPHSLIDERHORIZONTALGRIP_HXX
#define __APPS_INCLUDE_GRAPHICS_NXWIDGETS_CGLYPHSLIDERHORIZONTALGRIP_HXX

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "graphics/nxwidgets/csliderhorizontalgrip.hxx"
#include "graphics/nxwidgets/cimage.hxx"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Implementation Classes
 ****************************************************************************/

#if defined(__cplusplus)

namespace NXWidgets
{
  class IBitmap;

  /**
   * Class representing the grip, or draggable button, with a horizontal
   * slider.
   */

  class CGlyphSliderHorizontalGrip : public CImage
  {
  protected:

    /**
     * The CImage to represent the grip.
     */

    //CImage* m_pImage;

    /**
     * Draw the area of this widget that falls within the clipping region.
     * Called by the redraw() function to draw all visible regions.
     *
     * @param port The CGraphicsPort to draw to.
     * @see redraw()
     */

    //virtual void drawContents(CGraphicsPort *port);

    /**
     * Draw the area of this widget that falls within the clipping region.
     * Called by the redraw() function to draw all visible regions.
     *
     * @param port The CGraphicsPort to draw to.
     * @see redraw()
     */

    //virtual void drawBorder(CGraphicsPort *port);

    /**
     * Resize the slider to the new dimensions.
     *
     * @param width The new width.
     * @param height The new height.
     */

    virtual void onResize(nxgl_coord_t width, nxgl_coord_t height);

    /**
     * Starts dragging the grip and redraws it.
     *
     * @param x The x coordinate of the click.
     * @param y The y coordinate of the click.
     */

    virtual void onClick(nxgl_coord_t x, nxgl_coord_t y);

    /**
     * Redraws the grip.
     *
     * @param x The x coordinate of the mouse.
     * @param y The y coordinate of the mouse.
     */

    virtual void onRelease(nxgl_coord_t x, nxgl_coord_t y);

    /**
     * Redraws the grip.
     *
     * @param x The x coordinate of the mouse.
     * @param y The y coordinate of the mouse.
     */

    virtual void onReleaseOutside(nxgl_coord_t x, nxgl_coord_t y);

    /**
     * Moves the grip to follow the mouse.
     *
     * @param x The x coordinate of the mouse.
     * @param y The y coordinate of the mouse.
     * @param vX The horizontal distance of the drag.
     * @param vY The vertical distance of the drag.
     */

    virtual void onDrag(nxgl_coord_t x, nxgl_coord_t y,
                        nxgl_coord_t vX, nxgl_coord_t vY);

  public:

    /**
     * Constructor.
     *
     * @param pWidgetControl The controlling widget for the display
     * @param x The x coordinate of the grip, relative to its parent.
     * @param y The y coordinate of the grip, relative to its parent.
     * @param width The width of the grip.
     * @param height The height of the grip.
     */

    CGlyphSliderHorizontalGrip(CWidgetControl *pWidgetControl,
                          nxgl_coord_t x, nxgl_coord_t y,
                          nxgl_coord_t width, nxgl_coord_t height,
                          IBitmap *pGripBitmap);


    /**
     * Destructor.
     */

    virtual inline ~CGlyphSliderHorizontalGrip(void) { }
  };
}

#endif // __cplusplus

#endif // __APPS_INCLUDE_GRAPHICS_NXWIDGETS_CGLYPHSLIDERHORIZONTALGRIP_HXX
