/****************************************************************************
 **
 ** Copyright (C) 2012 Nokia Corporation and/or its subsidiary(-ies).
 ** All rights reserved.
 ** Contact: Nokia Corporation (qt-info@nokia.com)
 **
 ** This file is part of the examples of the Qt Toolkit.
 **
 ** $QT_BEGIN_LICENSE:BSD$
 ** You may use this file under the terms of the BSD license as follows:
 **
 ** "Redistribution and use in source and binary forms, with or without
 ** modification, are permitted provided that the following conditions are
 ** met:
 **   * Redistributions of source code must retain the above copyright
 **     notice, this list of conditions and the following disclaimer.
 **   * Redistributions in binary form must reproduce the above copyright
 **     notice, this list of conditions and the following disclaimer in
 **     the documentation and/or other materials provided with the
 **     distribution.
 **   * Neither the name of Nokia Corporation and its Subsidiary(-ies) nor
 **     the names of its contributors may be used to endorse or promote
 **     products derived from this software without specific prior written
 **     permission.
 **
 ** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 ** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 ** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 ** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 ** OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 ** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 ** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 ** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 ** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 ** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 ** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
 ** $QT_END_LICENSE$
 **
 ****************************************************************************/

#include <QtGui>
#include <math.h>

#include "coloritem.h"
#include "GraphicsView.h"
#include "MainWindow.h"

#define DEBUG 0

GraphicsView::GraphicsView(QWidget *parent) : QGraphicsView(parent), scene(-200, -200, 400, 400){
  // Addition of the ball
  item = new ColorItem;
  item->setPos(0,0);
  scene.addItem(item);

  // Addition of Lines
  vert = new QGraphicsLineItem(0, -200, 0, 200, 0);
  scene.addItem(vert);
  hor = new QGraphicsLineItem(-200, 0, 200, 0, 0);
  scene.addItem(hor);

  // Refresh scene items
  setScene(&scene);

  // Initial state of position
  xval = yval = 0;
}

GraphicsView::~GraphicsView() {
}

void GraphicsView::resizeEvent(QResizeEvent *event){
  event = event;
  fitInView(sceneRect());
}

void GraphicsView::mouseMoveEvent(QMouseEvent *event){
  QPointF pos = mapToScene(event->pos());                               // Get the position of the mouse pointer
  QRectF rect = mapToScene(this->viewport()->rect()).boundingRect();    // Get the dimensions of the scene

  // Limit the movement of the item to the bounderies of the scene
  if (!rect.contains(pos)){  
    pos.setX(qMin(rect.right(), qMax(pos.x(), rect.left()))); 
    pos.setY(qMin(rect.bottom(), qMax(pos.y(), rect.top()))); 
  }

  item->setPos(pos);                            // Refresh the item position in scene
  xval = -pos.y()/2; yval = pos.x()/2;          // Refresh the values of X and Y

  // Protection of X and Y values in order to not send values not recognized by the motor
  xval = xval > 100 ? 100 : xval;
  xval = xval < -100 ? -100 : xval;
  yval = yval > 100 ? 100 : yval;
  yval = yval < -100 ? -100 : yval;

  if (DEBUG)
    fprintf(stderr, "xval: %3d - yval: %3d\n", xval, yval);
}

void GraphicsView::mouseReleaseEvent(QMouseEvent *event){
  event = event;      // Event will not be used; to delete warning
  QPointF pos(0,0);   // Definition of new pointer
  item->setPos(pos);  // Refresh of item position acording to new pointer

  xval = 0; yval = 0; // Refresh of X and Y values

  if (DEBUG)
    fprintf(stderr, "xval: %3d - yval: %3d\n", xval, yval);
}
