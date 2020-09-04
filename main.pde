class node
{
  node parent;
  int x, y, state, index;
  int h, g, t;

  node()
  {
    parent=this;
    state=0;
    index=0;
  }

  void setUp(int x_, int y_, int g_, int h_, int t_, node parent_)
  {
    parent = parent_;
    x=x_;
    y=y_;
    g=g_;
    h=h_;
    t=t_;
    state=1;
  }

  void setUp(int g_, int h_, int t_, node parent_)
  {
    parent = parent_;
    g=g_;
    h=h_;
    t=t_;
    state=1;
  }

  boolean update(int g_, int h_, int t_, node p_node) {

    if (t < t_)
      return false;
    else if (t>t_) {
      setUp(g_, h_, t_, p_node);
      return true;
    } else if (g < g_)
      return false;
    else if (g > g_) {
      setUp(g_, h_, t_, p_node);
      return true;
    } else if (h < h_)
      return false;
    else {
      setUp(g_, h_, t_, p_node);
      return true;
    }
  }

  boolean isBetterThan(node p_node) {
    if (t < p_node.t)
      return true;
    else if (t>p_node.t)
      return false;

    else if (g < p_node.g)
      return true;
    else if (g > p_node.g)
      return false;

    else if (h < p_node.h)
      return true;
    else
      return false;
  }

  void use() {
    state=2;
  }

  boolean isUsed() {
    return state==2;
  }

  boolean initialized() {
    return state==1;
  }
}

class ASTAR extends Thread
{
  int wid, hei, gx, gy, sx, sy;
  int map[][], h_map[][];
  node node_map[][];

  PImage obs, scan;

  //anlyzing
  int x, y;
  //int g_cost[] = {10, 14, 10, 14,10, 14,10, 14}; 
  //int dir[][] = {{1, 0}, {1, 1}, {0, 1}, {-1, 1}, {-1, 0}, {-1, -1}, {0, -1}, {1, -1}};
  
  //experimental
  int g_cost[] = {20, 22, 28, 22, 20, 22, 28, 22, 20, 22, 28, 22, 20, 22, 28, 22}; 
  int dir[][] = {{2, 0 }, {2, 1}, {2, 2}, {1, 2}, {0, 2}, {-1, 2}, {-2, 2}, {-2, 1}, {-2, 0}, {-2, -1}, {2, -2}, {-1, -2}, {0, -2}, {1, -2}, {2, -2}, {2, -1}};
  
  ArrayList<node> unused = new ArrayList<node>();
  node center_node, start_node, end_node;

  int time_cost = 0;

  ASTAR(int w_, int h_) {
    wid=w_;
    hei=h_;

    map = new int[wid][hei];
    h_map = new int[wid][hei];

    obs = createImage(wid, hei, ARGB);
    scan = createImage(wid, hei, ARGB);

    node_map = new node[wid][hei];
    for (int x=0; x<wid; x++) 
      for (int y=0; y<hei; y++) 
        node_map[x][y] = new node();
  }

  //for real-time update
  void run()
  {
    while (true)
      if (checkPath(end_node))
      {
        resetCalc();
        anylize();
      }
  }

  void resetCalc() {
    unused.clear();

    scan = createImage(wid, hei, ARGB);

    node_map = new node[wid][hei];
    for (int x=0; x<wid; x++) 
      for (int y=0; y<hei; y++) 
        node_map[x][y] = new node();

    x=sx;
    y=sy;
    center_node = node_map[x][y];
    start_node = center_node;
  }

  void resetMap() {
    unused.clear();
    obs = createImage(wid, hei, ARGB);
    scan = createImage(wid, hei, ARGB);
    node_map = new node[wid][hei];
    for (int x=0; x<wid; x++) 
      for (int y=0; y<hei; y++) 
        node_map[x][y] = new node();
        
    map = new int[wid][hei];
  }

  void show() {
    background(255);
    image(scan, 0, 0);
    image(obs, 0, 0);
    fill(0);
    text(time_cost, 10, 10);
  }

  void details(int x, int y) {
    node curr = node_map[x][y];
    if (curr!=null)
      print("g : "+ curr.g+"    h : "+curr.h+"     g+h : "+curr.t+"\r\n");
  }

  void drawObs(int x_, int y_, int r) {
    obs.loadPixels();
    for (int x=x_-r; x<x_+r; x++) {
      if (x<0||x>=wid)
        continue;
      for (int y=y_-r; y<y_+r; y++) {
        {
          if (y<0||y>=hei)
            continue;
          obs.pixels[y*wid+x]=color(0, 0, 0);
          map[x][y]=5;
        }
      }
    }
    obs.updatePixels();
  }

  void eraseObs(int x_, int y_, int r) {
    obs.loadPixels();
    for (int x=x_-r; x<x_+r; x++) {
      if (x<0||x>=wid)
        continue;
      for (int y=y_-r; y<y_+r; y++) {
        {
          if (y<0||y>=hei)
            continue;
          obs.pixels[y*wid+x]=color(0, 0, 0, 0);
          map[x][y]=0;
        }
      }
    }
    obs.updatePixels();
  }

  void startDraw() {
    scan.loadPixels();
  }
  void stopDraw() {
    scan.updatePixels();
  }

  void drawDot(int x, int y, int r, int g, int b) {
    scan.pixels[y*wid+x]=color(r, g, b);
  }

  void setGoal(int x_, int y_) {
    gx=x_;
    gy=y_;

    startDraw();
    drawDot(x_, y_, 0, 0, 255);
    stopDraw();

    for (int x=0; x<wid; x++) 
      for (int y=0; y<hei; y++) 
        h_map[x][y] = h_cost(x, y);
  }

  void setStart(int x_, int y_) {
    sx=x_;
    sy=y_;

    startDraw();
    drawDot(x_, y_, 0, 0, 255);
    stopDraw();

    x=sx;
    y=sy;
    center_node = node_map[x][y];
    start_node = center_node;
  }

  int h_cost(int x_, int y_) {
    //return ((abs(x_-gx)+1)*(abs(y_-gy)+1)+(abs(x_-gx))+(abs(y_-gy)))*10;
    return ((abs(x_-gx))+(abs(y_-gy)))*10;  //*10 due to the base scale of the g cost
    //return mag(x_-gx, y_-gy);
    //return (int)sqrt(((x_-gx)*(x_-gx)+(y_-gy)*(y_-gy))*100.0);
    //return (x_-gx)*(x_-gx)+(y_-gy)*(y_-gy);
  }

  void addWithSort(ArrayList<node> array, node insert) {
    int size = array.size();

    for (int i=0; i<size; i++) {
      if (insert.isBetterThan(array.get(i)))
      {
        insert.index=i;
        array.add(i, insert);
        return;
      }
    }

    insert.index=size;
    array.add(insert);
  }

  void merge(ArrayList<node> src, ArrayList<node> dst, int start_index) {
    int  size_src = src.size();

    if (size_src==0)
      return;

    int index=max(start_index, 0), 
      size = dst.size();

    if (size>0) {

      node node_src = src.get(0);

      for (; index>0 && node_src.isBetterThan(dst.get(index)); index--);

      for (; index<size; index++) {
        if (node_src.isBetterThan(dst.get(index))) {

          node_src.index = index;
          dst.add(index, node_src);
          src.remove(0);

          if (src.size()==0)return;

          node_src = src.get(0);
        }
      }
    }
    dst.addAll(src);
  }
  
  /*void merge_(ArrayList<node> src, ArrayList<node> dst, int start_index) {
    int  size_src = src.size();

    if (size_src==0)
      return;

    int index=max(start_index, 0), 
      size = dst.size(),
      i_src=0;

    if (size>0) {

      node node_src = src.get(i_src);

      for (; index>0 && node_src.isBetterThan(dst.get(index)); index--);

      for (; index<size; index++) {
        if (node_src.isBetterThan(dst.get(index))) {

          node_src.index = index;
          dst.add(index, node_src);

          if (++i_src==size_src)return;

          node_src = src.get(i_src);
        }
      }
    }
    dst.addAll(src);
  }*/

  void drawPath() {

    startDraw();
    node curr = end_node;
    if (end_node.parent != end_node)
      while (curr != start_node) {
        drawDot(curr.x, curr.y, 20, 20, 200);
        curr=curr.parent;
      }
    stopDraw();
  }

  boolean checkPath(node curr) {
    if (map[curr.x][curr.y]>0)
      return true;
    else if (curr == start_node)
      return false;
    else
      return checkPath(curr.parent);
  }

  void anylize() {
    startDraw();
    int t_start=millis();

    int h, g, t;
    int dx, dy, i, s_i=0;

    while (true) {
      if (unused.size()>0) {
        center_node=unused.get(0);
        unused.remove(0);
        center_node.use();

        s_i++;

        x=center_node.x;
        y=center_node.y;

        if (unused.size()==0)
          break;
      }

      if (x==gx && y==gy ) {
        end_node = center_node;
        break;
      }
      ArrayList<node> partial = new ArrayList<node>();

      for (i=0; i<12; i++) {
        dx=x+dir[i][0];
        dy=y+dir[i][1];

        if (dx<0 || dx>=wid || dy<0 || dy>=hei || map[dx][dy]!=0)
          continue;

        h=h_map[dx][dy];
        g=g_cost[i]+center_node.g;

        t=g+h;

        node curr = node_map[dx][dy];

        if (curr.state == 0) {
          curr.setUp(dx, dy, g, h, t, center_node);
          node_map[dx][dy] = curr;
          drawDot(dx, dy, g, h, 0);
          addWithSort(partial, curr);
        } else if (curr.state == 1 && curr.update(g, h, t, center_node)) {
          addWithSort(partial, curr);
          drawDot(dx, dy, g, h, 0);
          unused.remove(curr);
        }
      }
      merge(partial, unused, center_node.index-s_i);
    }

    stopDraw();

    time_cost = millis()-t_start;

    drawPath();
  }

  int anylize_step() {
    startDraw();

    int h, g, t;
    int dx, dy, i;

    if (unused.size()>0) {
      center_node=unused.get(0);
      unused.remove(0);
      center_node.use();

      x=center_node.x;
      y=center_node.y;
      print(x+"   "+y+"\r\n");

      if (unused.size()==0)
        return 1;
    }

    if (x==gx && y==gy ) {
      end_node = center_node;
      return 1;
    }
    ArrayList<node> partial = new ArrayList<node>();

    for (i=0; i<8; i++) {
      dx=x+dir[i][0];
      dy=y+dir[i][1];

      if (dx<0 || dx>=wid || dy<0 || dy>=hei || map[dx][dy]!=0)
        continue;

      h=h_map[dx][dy];
      g=g_cost[i]+center_node.g;
      t=g+h;

      node curr = node_map[dx][dy];

      if (curr.state == 0) {
        curr.setUp(dx, dy, g, h, t, center_node);
        addWithSort(partial, curr);
        drawDot(dx, dy, g, h, 0);
      } else if (curr.state == 1 && curr.update(g, h, t, center_node)) {
        addWithSort(partial, curr);
        drawDot(dx, dy, g, h, 0);
        unused.remove(curr);
      }
    }

    merge(partial, unused, unused.indexOf(center_node));
    stopDraw();

    return 0;
  }
}
