/*
A-star path finding algorithm
by Brian Tu (RTU)

Just an experimental work for knowing how this algorithm works.
*/

ASTAR astar = new ASTAR(600, 600);  //the size have to be the same as the window

void setup()
{
  size(600, 600);
  background(255);

  astar.setGoal(500, 500);
  astar.setStart(50, 50);
  astar.anylize();
  astar.start();
}

void draw()
{
  if (mousePressed)
    if (mouseButton == LEFT)
      astar.drawObs(mouseX, mouseY, 5);
    else if (mouseButton == RIGHT)
      astar.eraseObs(mouseX, mouseY, 5);

  if (keyPressed && key=='a') {
    stroke(255, 0, 0);
    for (int i=0; i<100; i++)
    {
      if (astar.anylize_step()==1) {
        noLoop();
        break;
      }
    }
  }

  astar.show();
}

void keyPressed()
{
  if (key=='r') 
    astar.resetMap();
}

void mouseClicked()
{
  if (mouseButton == CENTER) {
    astar.details(mouseX, mouseY);
  }
}
