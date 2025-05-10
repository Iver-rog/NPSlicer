
// struct RingBuffer<T:std::marker::Copy,const S:usize>{
struct RingBuffer<T,const S:usize>{
    buffer:[Option<T>;S],
    pntr: usize,
}
impl<T:std::marker::Copy,const S:usize> RingBuffer<T,S>{
// impl<T,const S:usize> RingBuffer<T,S>{
    fn new() -> Self {
        Self{
            buffer:[None;S],
            pntr: 0,
        }
    }
    fn advance(&mut self, item:T)->Option<T>{
        let res = std::mem::replace(&mut self.buffer[self.pntr],Some(item));
        self.pntr = (self.pntr+1)%S;
        return res
    }
    fn take(&mut self)->Option<T>{
        let res = std::mem::replace(&mut self.buffer[self.pntr],None);
        self.pntr = (self.pntr+1)%S;
        return res
    }
}
#[test]
fn ring_buffer_test(){
    let mut buf:RingBuffer<usize,2> = RingBuffer::new();
    assert_eq!(buf.advance(1),None);
    assert_eq!(buf.advance(2),None);
    assert_eq!(buf.advance(3),Some(1));
    assert_eq!(buf.advance(4),Some(2));
    assert_eq!(buf.advance(5),Some(3));
    assert_eq!(buf.advance(6),Some(4));
    assert_eq!(buf.take(),Some(5));
    assert_eq!(buf.take(),Some(6));
}
//
//
// struct Buffer<Iter,const S:usize> 
// where Iter: Iterator {
//     iter: Iter,
//     buffer:RingBuffer<Iter::Item,S>
// }
// impl<Iter,const S:usize> Iterator for Buffer<Iter,S> 
// where Iter: Iterator {
//     type Item = Iter::Item;
//     fn next(&mut self) -> Option<Self::Item> {
//         let v = self.iter.next();
//         let next = std::mem::replace(&mut self.buffer[self.pntr],v);
//         next
//     }
// }

