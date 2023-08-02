import asyncio
import cv2
import soccerxcomm as sdk

async def main():
    client = sdk.Client("localhost", 14514, 14515, "r1")

    await client.connect()

    await asyncio.sleep(2)

    captured_image = await client.get_capture_image()

    print('captured_image')
    cv2.imshow('img',captured_image)
    cv2.waitKey(0)
    await client.disconnect()

if __name__ == '__main__':
    asyncio.run(main())
    print("done")

