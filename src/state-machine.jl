using LinearAlgebra
# This component is a multi-DM channel mixer and outer for an ALPAO DM.

# Channels are created dynamically when we receive a command with a new description
# Channels are "sticky" in that the most recent command is kept until changed
# The values of all channels are summed together along with the bestflat
# and placed into dither_detpoint.

const DType = Float32

# These are our variables for the service plus a state machine context to keep
# track of our current state.
Base.@kwdef mutable struct SSCSm <: Hsm.AbstractStateMachine
    # This tracks the state of the block
    context::Hsm.StateMachineContext=Hsm.StateMachineContext()

    ####### These are our set-able variables
    fulldh::Bool=true
    # The reference image is a vector of just the valid pixels
    reference_image::Matrix{DType}=zeros(DType,0,0)
    reference_image_set::Bool=false
    # A matrix of valid pixel locations
    image_mask::BitMatrix=BitMatrix([;;])
    image_mask_set::Bool=false
    # Mask to use for a phase correction
    image_mask_phase::BitMatrix=BitMatrix([;;])
    image_mask_phase_set::Bool=false
    # Mask to use for an amplitude correction
    image_mask_amplitude::BitMatrix=BitMatrix([;;])
    image_mask_amplitude_set::Bool=false
    # N_valid_pixels * N_actus
    image_to_actus::Matrix{DType}=zeros(DType,0,0)
    image_to_actus_set::Bool=false
    
    ######## These are our working variables / scratch spaces
    # Full image scractch space to work in
    image_scratch::Matrix{DType}=zeros(DType,0,0)
    # Vector of valid pixels (of correct length = count of true in image_mask) for us to work on
    image_scratch_valid::Vector{DType}=zeros(DType,0)
    # We go from image directly to actuators
    actus_coefficients::Vector{DType}=zeros(DType,0)

    output_channel::Union{Nothing,Aeron.AeronPublication}=nothing
    output_buffer::Vector{UInt8}=zeros(UInt8,2112)

end

# These variables are the externally visible state 
# that will be reported to any listeners when requested.
RTCBlock.statevariables(sm::SSCSm) = (;
    sm.fulldh,
    reference_image        = sm.reference_image_set ? sm.reference_image : nothing,
    image_mask             = sm.image_mask_set ? UInt8.(sm.image_mask) : nothing,
    image_mask_phase       = sm.image_mask_phase_set ? UInt8.(sm.image_mask_phase) : nothing,
    image_mask_amplitude   = sm.image_mask_amplitude_set ? UInt8.(sm.image_mask_amplitude) : nothing,
    image_to_actus         = sm.image_to_actus_set ? sm.image_to_actus : nothing,
)

const sm = SSCSm()

include("generic-state-machine.jl")


Hsm.on_entry!(sm, :Waiting) do 
    sm.fulldh=true
    sm.reference_image_set = false
    sm.image_mask_set = false
    sm.image_mask_phase_set = false
    sm.image_mask_amplitude_set = false
    sm.image_to_actus_set = false
    return
end


Hsm.on_event!(sm, :Top, :fulldh) do payload
    event_info = EventMessage(payload, initialize=false)
    sm.fulldh = getargument(Float64, event_info)
    return Hsm.Handled
end
Hsm.on_event!(sm, :Top, :reference_image) do payload
    event_info = EventMessage(payload, initialize=false)
    # The value is stored as an array message inside
    payload = getargument(ArrayMessage{Float32,2}, event_info)
    sm.reference_image = collect(arraydata(payload)) # allocates
    sm.reference_image_set = true
    put!(event_queue, :StartIfReady)
    return Hsm.Handled
end
# Receive properties
Hsm.on_event!(sm, :Top, :image_mask) do payload
    event_info = EventMessage(payload, initialize=false)
    # The value is stored as an array message inside
    payload = getargument(ArrayMessage{UInt8,2}, event_info)
    sm.image_mask = BitMatrix(arraydata(payload) .!=0) # allocates
    sm.image_mask_set = true

    # Allocate our scratch spaces based on the number of pixels
    # in this mask
    sm.image_scratch_valid = zeros(DType,count(sm.image_mask))
    # Image scratch space for the COG algorithm to work on
    sm.image_scratch = zeros(DType,size(sm.image_mask))

    put!(event_queue, :StartIfReady)
    return Hsm.Handled
end
Hsm.on_event!(sm, :Top, :image_mask_phase) do payload
    event_info = EventMessage(payload, initialize=false)
    # The value is stored as an array message inside
    payload = getargument(ArrayMessage{UInt8,2}, event_info)
    sm.image_mask_phase = BitMatrix(arraydata(payload) .!=0) # allocates
    sm.image_mask_phase_set = true

    put!(event_queue, :StartIfReady)
    return Hsm.Handled
end
Hsm.on_event!(sm, :Top, :image_mask_amplitude) do payload
    event_info = EventMessage(payload, initialize=false)
    # The value is stored as an array message inside
    payload = getargument(ArrayMessage{UInt8,2}, event_info)
    sm.image_mask_amplitude = BitMatrix(arraydata(payload) .!=0) # allocates
    sm.image_mask_amplitude_set = true

    put!(event_queue, :StartIfReady)
    return Hsm.Handled
end
Hsm.on_event!(sm, :Top, :image_to_actus) do payload
    event_info = EventMessage(payload, initialize=false)
    # The value is stored as an array message inside
    payload = getargument(ArrayMessage{Float32,2}, event_info)
    sm.image_to_actus = collect(arraydata(payload)) # allocates
    sm.image_to_actus_set = true

    # We go from image to modes, then modes to actuators.
    sm.actus_coefficients = zeros(DType,size(sm.image_to_actus,2))

    put!(event_queue, :StartIfReady)
    return Hsm.Handled
end


Hsm.on_event!(sm, :Waiting, :StartIfReady) do payload
    if  sm.reference_image_set &&
        sm.image_mask_set &&
        sm.image_mask_phase_set &&
        sm.image_mask_amplitude_set &&
        sm.image_to_actus_set
        Hsm.transition!(sm, :Processing)
        return Hsm.Handled
    end
    return Hsm.NotHandled
end


Hsm.on_entry!(sm, :Ready) do 
    conf = AeronConfig(
        uri=ENV["PUB_DATA_URI"],
        stream=parse(Int, ENV["PUB_DATA_STREAM"]),
    )
    sm.output_channel = Aeron.publisher(aeron_ctx, conf)

    return 
end
Hsm.on_exit!(sm, :Ready) do
    if !isnothing(sm.output_channel)
        close(sm.output_channel)
        sm.output_channel = nothing
    end
    return
end

# When stopping send a zero command once
Hsm.on_entry!(sm, :Stopped) do 
    fill!(sm.actus_coefficients,0)

    output_msg = ArrayMessage{DType,1}(sm.output_buffer)
    arraydata!(output_msg, sm.actus_coefficients)

    output_msg.header.TimestampNs = round(UInt64, time()*1e9)
    status = Aeron.publication_offer(sm.output_channel, view(sm.output_buffer, 1:sizeof(sm.output_buffer)))
    if status == :adminaction
        @warn lazy"could not publish wavefront measurement ($status)"
    elseif status != :success && status != :backpressured && status != :notconnected
        error(lazy"could not publish wavefront sensor measurement ($status)")
    end
end

Hsm.on_exit!(sm, :Processing) do 
    sm.reference_image_set=false
    sm.image_mask_set=false
    sm.cog_image_mask_set=false
    sm.image_to_modes_set=false
    sm.modes_to_actus_set=false
    sm.slopes_to_TT_set=false
    sm.cog_reference_set=false
end

Hsm.on_entry!(sm, :Paused) do 
end


Hsm.on_event!(sm, :Paused, :Data) do payload
    return Hsm.Handled
end

Hsm.on_event!(sm, :Playing, :Data) do payload
    # Decode message
    msg_in = ArrayMessage{DType,2}(payload)
    msg_in_dat = arraydata(msg_in)
    sm.image_scratch .= msg_in_dat

    # Collect just the valid unmasked pixels by iterating through
    i_valid = 0
    for I_px in eachindex(sm.image_mask, msg_in_dat)
        if !sm.image_mask[I_px]
            continue
        end
        i_valid += 1
        sm.image_scratch_valid[i_valid] = msg_in_dat[I_px]
    end


    # Subtract the reference image
    sm.image_scratch_valid .-= sm.reference_image

    if sm.fulldh
        # FDH
        for I_px in eachindex(sm.image_mask_phase)
            if !sm.image_mask_phase[I_px]
                sm.image_scratch_valid[I_px] = 0
            end
        end
    else
        # HDH
        for I_px in eachindex(sm.image_mask_amplitude)
            if !sm.image_mask_amplitude[I_px]
                sm.image_scratch_valid[I_px] = 0
            end
        end
    end

    # Use interaction matrix to determine the linear response of the sensor
    # i.e. pixels -> modes
    mul!(sm.actus_coefficients, sm.image_to_actus', sm.image_scratch_valid)


    # TODO: safeing criteria for SCC if signal is lost. What should we use?
    # if source_mean_blob < 1#50
    #     println("blob too faint")
    #     # Publish our measurement in actuator space
    #     output_msg.header.TimestampNs = msg_in.header.TimestampNs
        
    #     status = Aeron.publication_offer(publication, view(sm.output_buffer, 1:sizeof(sm.output_buffer)))
    #     if status == :adminaction
    #         @warn lazy"could not publish wavefront measurement ($status)"
    #     elseif status != :success && status != :backpressured && status != :notconnected
    #         error(lazy"could not publish wavefront sensor measurement ($status)")
    #     end
    #     return Hsm.Handled
    # end


    # Publish our measurement in actuator space
    output_msg = ArrayMessage{DType,1}(sm.output_buffer)
    arraydata!(output_msg, sm.actus_coefficients)
    output_msg.header.TimestampNs = msg_in.header.TimestampNs
    
    status = Aeron.publication_offer(sm.output_channel, view(sm.output_buffer, 1:sizeof(sm.output_buffer)))
    if status == :adminaction
        @warn lazy"could not publish wavefront measurement ($status)"
    elseif status != :success && status != :backpressured && status != :notconnected
        error(lazy"could not publish wavefront sensor measurement ($status)")
    end
    return Hsm.Handled
end


function cog(img)
    tot = zero(eltype(img))
    X = zero(eltype(img))
    Y = zero(eltype(img))
    for xi in axes(img,1), yi in axes(img,2)
        X += xi*img[xi,yi]
        Y += yi*img[xi,yi]
        tot += img[xi,yi]
    end
    return X/tot, Y/tot
end