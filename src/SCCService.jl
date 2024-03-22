module SCCService
using Hsm
using Aeron
using RTCBlock
using StaticStrings
using SpidersMessageEncoding

aeron_ctx::AeronContext = AeronContext()
const event_queue::Channel{Symbol} = Channel{Symbol}(10)

include("state-machine.jl")

function main(ARGS=[])
    global aeron_ctx = AeronContext()
    RTCBlock.serve(sm, event_queue, aeron=aeron_ctx, )
end

end