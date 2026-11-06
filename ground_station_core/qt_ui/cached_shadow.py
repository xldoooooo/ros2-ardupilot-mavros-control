"""为不透明圆角表面缓存 Qt 阴影，缩放时仅拼接边角纹理。"""

from __future__ import annotations

from functools import lru_cache
from math import ceil

from PySide6.QtCore import QPoint, QPointF, QRectF, Qt
from PySide6.QtGui import QColor, QImage, QPainter, QPainterPath, QPixmap, QTransform
from PySide6.QtWidgets import (
    QGraphicsDropShadowEffect, QGraphicsEffect, QGraphicsPathItem, QGraphicsScene,
)


@lru_cache(maxsize=32)
def _shadow_texture(
    radius: float, offset_x: float, offset_y: float, rgba: int,
    corner: float, dpr: float, height: float,
) -> tuple[QPixmap, float, float, float, float, float]:
    """以原生 Qt 模糊生成小纹理；同参数窗口共享缓存。"""
    # 留出完整卷积范围，中心留有直边供缩放时延展。
    pad = ceil((radius + max(abs(offset_x), abs(offset_y)) + 2) * dpr) / dpr
    edge = ceil(corner + radius + 2)
    side = edge * 2 + 4
    height = min(height, side)
    scene = QGraphicsScene()
    path = QPainterPath()
    path.addRoundedRect(QRectF(0, 0, side, height), corner, corner)
    item = QGraphicsPathItem(path)
    item.setPen(Qt.PenStyle.NoPen)
    item.setBrush(Qt.GlobalColor.black)
    scene.addItem(item)
    effect = QGraphicsDropShadowEffect()
    effect.setBlurRadius(radius)
    effect.setOffset(offset_x, offset_y)
    effect.setColor(QColor.fromRgba(rgba))
    item.setGraphicsEffect(effect)
    extent = side + 2 * pad
    extent_y = height + 2 * pad
    image = QPixmap(ceil(extent * dpr), ceil(extent_y * dpr))
    image.setDevicePixelRatio(dpr)
    image.fill(Qt.GlobalColor.transparent)
    painter = QPainter(image)
    painter.setRenderHint(QPainter.RenderHint.Antialiasing)
    scene.render(painter, QRectF(0, 0, extent, extent_y), QRectF(-pad, -pad, extent, extent_y))
    painter.end()

    # Qt 投影同时画出源图；恢复源图下方的阴影 alpha，防止圆角叠画变暗。
    result = image.toImage().convertToFormat(QImage.Format.Format_RGBA8888)
    # 保持 effect 的设备像素留边不变；直接禁用会改变分数 DPI 的源图对齐。
    effect.setColor(QColor(0, 0, 0, 0))
    image.fill(Qt.GlobalColor.transparent)
    painter = QPainter(image)
    painter.setRenderHint(QPainter.RenderHint.Antialiasing)
    scene.render(painter, QRectF(0, 0, extent, extent_y), QRectF(-pad, -pad, extent, extent_y))
    painter.end()
    mask = image.toImage().convertToFormat(QImage.Format.Format_RGBA8888)
    color = QColor.fromRgba(rgba)
    pixels, source_pixels = result.bits(), mask.constBits()
    rgb = bytes((color.red(), color.green(), color.blue()))
    # 分数 DPI 的源图可能落在半像素位置，使用相同变换的掩膜恢复 alpha。
    for index in range(3, result.sizeInBytes(), 4):
        source_alpha = source_pixels[index]
        pixels[index - 3:index] = rgb
        if source_alpha:
            alpha = (
                round((pixels[index] - source_alpha) * 255 / (255 - source_alpha))
                if source_alpha < 255 else 0
            )
            pixels[index] = max(0, min(255, alpha))
    return (
        QPixmap.fromImage(result), float(pad), float(edge),
        min(float(edge), height / 2), extent, extent_y,
    )


class CachedShadowEffect(QGraphicsEffect):
    """保持原生投影接口，以缓存的九宫格替代每帧整面模糊。"""

    def __init__(
        self, parent=None, *, blur: float = 30.0,
        offset: tuple[float, float] = (0.0, 3.0),
        color: QColor | None = None, corner: float = 8.0,
    ) -> None:
        """记录表面的圆角与原投影参数。"""
        super().__init__(parent)
        self._blur = blur
        self._offset = QPointF(*offset)
        self._color = QColor(color if color is not None else QColor(16, 30, 44, 118))
        self._corner = corner
        self._opacity = 1.0

    def blurRadius(self) -> float:  # noqa: N802
        """返回沿用的 Qt 模糊半径。"""
        return self._blur

    def offset(self) -> QPointF:
        """返回沿用的投影偏移。"""
        return QPointF(self._offset)

    def color(self) -> QColor:
        """返回当前可见投影颜色，包括动画透明度。"""
        color = QColor(self._color)
        color.setAlpha(round(color.alpha() * self._opacity))
        return color

    def setColor(self, color: QColor) -> None:  # noqa: N802
        """仅在色相变化时切换缓存，透明度动画复用原纹理。"""
        color = QColor(color)
        self._opacity = color.alphaF()
        color.setAlpha(255)
        if self._color != color:
            self._color = color
        self.update()

    def boundingRectFor(self, rect: QRectF) -> QRectF:  # noqa: N802
        """保持原投影外延，防止父控件裁剪光晕。"""
        return rect.united(rect.translated(self._offset).adjusted(
            -self._blur, -self._blur, self._blur, self._blur,
        ))

    def draw(self, painter: QPainter) -> None:
        """绘制缓存边角后绘制源表面，布局和输入路径保持不变。"""
        rect = self.sourceBoundingRect(Qt.CoordinateSystem.LogicalCoordinates)
        if not rect.isEmpty() and self._opacity > 0 and self._color.alpha() > 0:
            dpr = painter.device().devicePixelRatioF()
            texture, pad, edge, edge_y, width, height = _shadow_texture(
                self._blur, self._offset.x(), self._offset.y(),
                self._color.rgba(), self._corner, dpr,
                min(rect.height(), 2 * ceil(self._corner + self._blur + 2) + 4),
            )
            self._draw_tiles(
                painter, rect, texture, pad, edge, edge_y, width, height, dpr,
            )
        # 与原生投影一致地按设备像素放置源图，避免分数 DPI 的边框漂移。
        origin = QPoint()
        source = self.sourcePixmap(
            Qt.CoordinateSystem.DeviceCoordinates, origin,
            QGraphicsEffect.PixmapPadMode.PadToEffectiveBoundingRect,
        )
        painter.save()
        painter.setWorldTransform(QTransform())
        painter.drawPixmap(origin, source)
        painter.restore()

    def _draw_tiles(
        self, painter: QPainter, rect: QRectF, texture: QPixmap,
        pad: float, edge: float, edge_y: float,
        side: float, height: float, dpr: float,
    ) -> None:
        """保留圆角像素尺寸，仅延展四边中段。"""
        target = rect.adjusted(-pad, -pad, pad, pad)
        cap_x = min(pad + edge, target.width() / 2)
        cap_y = min(pad + edge_y, target.height() / 2)
        tx = (target.left(), target.left() + cap_x, target.right() - cap_x, target.right())
        ty = (target.top(), target.top() + cap_y, target.bottom() - cap_y, target.bottom())
        sx = (0.0, pad + edge, side - pad - edge, side)
        sy = (0.0, pad + edge_y, height - pad - edge_y, height)
        painter.save()
        painter.setOpacity(painter.opacity() * self._opacity)
        for row in range(3):
            for column in range(3):
                if row == column == 1:
                    continue
                if tx[column + 1] <= tx[column] or ty[row + 1] <= ty[row]:
                    continue
                painter.drawPixmap(
                    QRectF(tx[column], ty[row], tx[column + 1] - tx[column], ty[row + 1] - ty[row]),
                    texture,
                    QRectF(sx[column] * dpr, sy[row] * dpr,
                           (sx[column + 1] - sx[column]) * dpr,
                           (sy[row + 1] - sy[row]) * dpr),
                )
        painter.restore()
