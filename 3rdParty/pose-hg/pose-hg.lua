require 'paths'
paths.dofile('./pose-hg-demo/util.lua')
paths.dofile('./pose-hg-demo/img.lua')

-- Load pre-trained model
m = torch.load('../3rdParty/pose-hg/umich-stacked-hourglass/umich-stacked-hourglass.t7')


function extractValidJoint(hms, coords)
    local actThresh = 0.002

    for i = 1, coords:size()[1] do
        if hms[i]:mean() <= actThresh then
            coords[i][1] = -1
            coords[i][2] = -1
        end
    end

    return coords
end


function getPredHm(hms)
    if hms:size():size() == 3 then hms = hms:view(1, hms:size(1), hms:size(2), hms:size(3)) end

    -- Get locations of maximum activations
    local max, idx = torch.max(hms:view(hms:size(1), hms:size(2), hms:size(3) * hms:size(4)), 3)
    local preds = torch.repeatTensor(idx, 1, 1, 2):float()
    preds[{{}, {}, 1}]:apply(function(x) return (x - 1) % hms:size(4) + 1 end)
    preds[{{}, {}, 2}]:add(-1):div(hms:size(3)):floor():add(.5)

    return preds
end


function predict(im)
    local inp = im

    -- Get network output
    local out = m:forward(inp:view(1, 3, 256, 256):cuda())
    cutorch.synchronize()
    local hm = out[2][1]:float()
    hm[hm:lt(0)] = 0

--     -- Get predictions (hm and img refer to the coordinate space)
--     local preds_hm, preds_img = getPreds(hm, center, scale)
    local preds_hm = getPredHm(hm)
    preds_hm:mul(4) -- Change to input scale

    joint = extractValidJoint(hm, preds_hm[1])

    -- Display the result
--     local dispImg = drawSkeleton(inp, hm, preds_hm[1])
--     image.save('/home/takiyu/tmp/c.jpg', dispImg)
--     print(preds_hm[1])
--     w = image.display{image=dispImg,win=w}

    return joint
end

-- local im = image.load('/home/takiyu/tmp/b.jpg')
-- predict(im)
