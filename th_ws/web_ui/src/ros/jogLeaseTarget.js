// During a release tail, callers may still re-render with their last nonzero
// command. The lease publisher must keep ramping toward zero until retouched.
export function jogLeaseTarget(operatorHeld, cmd) {
  return operatorHeld ? cmd : { vx: 0, wz: 0 }
}
