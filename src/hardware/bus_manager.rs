use shared_bus::{AtomicCheckMutex, BusMutex};

pub struct BusProxy<'a, BusT> {
    mutex: &'a AtomicCheckMutex<BusT>,
}

impl<'a, BusT> BusProxy<'a, BusT> {
    pub fn lock<R, F>(&mut self, handler: F) -> R
    where
        F: FnOnce(&mut BusT) -> R,
    {
        self.mutex.lock(|bus| handler(bus))
    }
}

pub struct BusManager<BusT> {
    mutex: AtomicCheckMutex<BusT>,
}

impl<BusT> BusManager<BusT> {
    pub fn new(bus: BusT) -> BusManager<BusT> {
        BusManager {
            mutex: AtomicCheckMutex::create(bus),
        }
    }

    pub fn acquire_bus(&self) -> BusProxy<BusT> {
        BusProxy { mutex: &self.mutex }
    }
}
