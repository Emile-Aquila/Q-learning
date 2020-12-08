use std::io;
use rand::{Rng, random};
use std::cmp::max;
use std::collections::VecDeque;
use std::ops::Add;

#[derive(Copy, Clone)]
enum MAP{
    normal,
    hall,
    goal,
}

fn calc(m:&MAP) -> f64{//即時報酬
    match m{
        MAP::normal => -1.0,
        MAP::hall => -20.0,
        MAP::goal => 20.0,
    }
}
#[derive(Copy, Clone)]
struct envi{
    map: [[MAP;4];4],
    action:[i8;4],
    x_max:usize,
    y_max:usize,
    move_prob:f64,
}
//act:1^,-1v,2<,-2>
impl envi{
    fn check(self,x:i8,y:i8) -> bool{//in the grid?
        if x>=self.x_max as i8 || x<0 {
            return false;
        }
        if y>=self.y_max as i8 || y<0 {
            return false;
        }
        return true;
    }
    fn transit(self,x:usize,y:usize,act:i8) -> (usize,usize,f64){//今回はstateに依存しない
        let (mut nx,mut ny) = (x as i8,y as i8);
        match act {
            1 => ny += 1,
            -1 => ny += -1,
            2 => nx -= 1,
            -2 => nx += 1,
            _ => println!("Error!"),
        }
        let (mut xx,mut yy) = (0,0);
        if self.check(nx, ny) {
            xx += nx as usize;
            yy += ny as usize;
        } else {
            xx += x as usize;
            yy += y as usize;
        }
        let val = calc(&self.map[xx][yy]);
        (xx,yy,val)
    }

    fn move_act(self,x:i8,y:i8,act:i8) -> (usize,usize,f64){
        let mut v= Vec::new();
        for (_,id) in self.action.iter().enumerate() {
            if *id == act{
                v.push((*id,self.move_prob));
            }else if *id != act*(-1){
                v.push((*id,(1.0-self.move_prob)/2.0));
            }
        }
        let mut sum = 0.0;
        let mut act_n = 0 as i8;
        let mut rng = rand::thread_rng();
        let tmp = rng.gen();
        for (_,(id,num)) in v.iter().enumerate(){
            sum += *num;
            if sum > tmp {
                act_n = *id;
                break;
            }
        }
        self.transit(x as usize,y as usize,act_n)
    }
}

#[derive(Copy, Clone)]
struct agent{
    Q:[[[f64;4];4];4],
    action:[i8;4],
    eps:f64,
    alp:f64,
    gmm:f64,
    x:usize,
    y:usize,
}

impl agent{
    fn init(mut self){
        self.x = 0;
        self.y = 0;
    }
    fn policy(self) -> i8{
        let mut rng = rand::thread_rng();
        if self.eps >= rng.gen(){//ランダムに動く
            self.action[(rng.gen::<u64>()%4) as usize]
        }else{//maxを返す
            let mut mx:f64 = self.Q[self.x][self.y][0];
            let mut idx = 1;
            for (ix,id) in self.action.iter().enumerate(){
                if mx < self.Q[self.x][self.y][ix] {
                    mx = self.Q[self.x][self.y][ix];
                    idx = *id;
                }
            }
            idx
        }
    }
    fn translate(self,act:i8) -> usize{
        match act {
            1 => 0,
            -1 => 1,
            2 => 2,
            -2 => 3,
            _ => {
                println!("Error!!!!");
                0
            }
        }
    }
    fn learn(mut self,env:&mut envi,mut max_time:u64){//ただしmax_timeは50の倍数でお願いします.
        max_time -= max_time %50;
        let mut deq:VecDeque<f64> = VecDeque::new();
        let mut sum = 0.0;
        for tim in 0..max_time {
            self.init();
            loop {
                let mut act = self.policy();
                let (mut xx,mut yy,mut delta) = env.transit(self.x,self.y,act);
                let mut mx = self.Q[xx][yy][0];
                for (_,id) in self.Q[xx][yy].iter().enumerate(){
                    if mx < *id {
                        mx = *id;
                    }
                }
                delta += mx*self.gmm;
                delta -= self.Q[self.x][self.y][self.translate(act)];
                self.Q[self.x][self.y][self.translate(act)] += delta*self.alp;
                self.x = xx;
                self.y = yy;
                let mut delta = calc(&mut env.map[self.x][self.y]);
                if delta == calc(&mut MAP::goal){
                    deq.push_back(1.0);
                    sum += 1.0;
                    if tim >= 50 {
                        sum -= *deq.front().unwrap();
                        deq.pop_front();
                        println!("try:{},score:{}",tim,sum/50.0);
                    }
                    break;
                }else if delta==calc(&mut MAP::hall){
                    deq.push_back(-1.0);
                    sum -= 1.0;
                    if tim >= 50 {
                        sum -= *deq.front().unwrap();
                        deq.pop_front();
                        println!("try:{},score:{}",tim,sum/50.0);
                    }
                    break;
                }
            }
        }
        for i in 0..4{//act:1^,-1v,2<,-2>
            for j in 0..4{
                print!("^:{},v:{},<:{},>:{}   ,",self.Q[j][i][0] as f64,
                       self.Q[j][i][1] as f64,self.Q[j][i][2] as f64,
                       self.Q[j][i][3] as f64);
            }
            println!(".");
        }
    }
}

fn main(){
    let mut env = envi{
        map: [
            [MAP::normal, MAP::normal, MAP::normal, MAP::hall],
            [MAP::normal, MAP::hall, MAP::normal, MAP::normal],
            [MAP::normal, MAP::normal, MAP::normal, MAP::normal],
            [MAP::normal, MAP::hall, MAP::hall, MAP::goal]
        ],
        action:[1,-1,2,-2],
        x_max:4,
        y_max:4,
        move_prob:0.8,
    };
    let mut player = agent{
        Q:[[[0.0;4];4];4],
        action:[1,-1,2,-2],
        eps:0.2,
        alp:0.3,
        gmm:0.9,
        x:0,
        y:0,
    };
    player.learn(&mut env,300);
}
